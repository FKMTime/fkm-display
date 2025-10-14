use alloc::rc::Rc;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    mutex::Mutex,
    semaphore::{GreedySemaphore, Semaphore},
};
use embedded_storage::{ReadStorage, Storage};
use portable_atomic::AtomicU8;
use tickv::{ErrorCode, FlashController};

const PART_OFFSET: u32 = 0x8000;
const PART_SIZE: u32 = 0xc00;

static mut NVS_READ_BUF: &mut [u8; 1024] = &mut [0; 1024];
static NVS_INSTANCES: AtomicU8 = AtomicU8::new(0);

pub struct Nvs {
    flash_peripheral: esp_hal::peripherals::FLASH<'static>,
    tickv: Rc<tickv::TicKV<'static, NvsFlash, 1024>>,
    semaphore: Rc<GreedySemaphore<CriticalSectionRawMutex>>,

    offset: usize,
    size: usize,
}

pub type Result<T> = core::result::Result<T, WmError>;

#[derive(Debug)]
pub enum WmError {
    FlashError(tickv::ErrorCode),
    SerdeError(serde_json::Error),
    TaskSpawnError,
    NvsError,
    Other,
}

impl From<tickv::ErrorCode> for WmError {
    fn from(value: tickv::ErrorCode) -> Self {
        Self::FlashError(value)
    }
}

impl From<serde_json::Error> for WmError {
    fn from(value: serde_json::Error) -> Self {
        Self::SerdeError(value)
    }
}

impl From<()> for WmError {
    fn from(_value: ()) -> Self {
        Self::Other
    }
}

impl From<core::convert::Infallible> for WmError {
    fn from(_value: core::convert::Infallible) -> Self {
        Self::Other
    }
}

impl Nvs {
    pub fn new(
        flash_offset: usize,
        flash_size: usize,
        flash: esp_hal::peripherals::FLASH<'static>,
    ) -> Result<Self> {
        if NVS_INSTANCES.load(core::sync::atomic::Ordering::Relaxed) > 0 {
            log::error!("Cannot spawn new NVS struct, clone original one instead!");
            return Err(WmError::NvsError);
        }

        NVS_INSTANCES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        unsafe { Self::new_unchecked(flash_offset, flash_size, flash) }
    }

    /// # Safety
    ///
    /// This is not checking if other nvs instance already exists (there should be only one nvs
    /// instancce!)
    pub unsafe fn new_unchecked(
        flash_offset: usize,
        flash_size: usize,
        flash: esp_hal::peripherals::FLASH<'static>,
    ) -> Result<Self> {
        let nvs = tickv::TicKV::<NvsFlash, 1024>::new(
            NvsFlash::new(flash_offset, unsafe { flash.clone_unchecked() }),
            unsafe { NVS_READ_BUF },
            flash_size,
        );
        nvs.initialise(hash(tickv::MAIN_KEY))?;

        Ok(Nvs {
            flash_peripheral: flash,
            tickv: Rc::new(nvs),
            semaphore: Rc::new(GreedySemaphore::new(1)),

            offset: flash_offset,
            size: flash_size,
        })
    }

    pub fn new_from_part_table(flash: esp_hal::peripherals::FLASH<'static>) -> Result<Self> {
        if let Some((offset, size)) =
            Self::read_nvs_partition_offset(unsafe { flash.clone_unchecked() })
        {
            Self::new(offset, size, flash)
        } else {
            log::error!("Nvs partition not found!");
            Err(WmError::NvsError)
        }
    }

    pub fn read_nvs_partition_offset(
        flash: esp_hal::peripherals::FLASH<'static>,
    ) -> Option<(usize, usize)> {
        let mut flash = esp_storage::FlashStorage::new(flash);

        let mut nvs_part = None;
        let mut bytes = [0xFF; 32];
        for read_offset in (0..PART_SIZE).step_by(32) {
            _ = flash.read(PART_OFFSET + read_offset, &mut bytes);
            if bytes == [0xFF; 32] {
                break;
            }

            let magic = &bytes[0..2];
            if magic != [0xAA, 0x50] {
                continue;
            }

            let p_type = &bytes[2];
            let p_subtype = &bytes[3];
            let p_offset = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
            let p_size = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
            //let p_name = core::str::from_utf8(&bytes[12..28]).unwrap();
            //let p_flags = u32::from_le_bytes(bytes[28..32].try_into().unwrap());
            //log::info!("{magic:?} {p_type} {p_subtype} {p_offset} {p_size} {p_name} {p_flags}");

            if *p_type == 1 && *p_subtype == 2 {
                nvs_part = Some((p_offset, p_size));
                break;
            }
        }

        nvs_part.map(|(offset, size)| (offset as usize, size as usize))
    }

    pub async fn get_key(&self, key: &[u8], buf: &mut [u8]) -> Result<()> {
        let _drop = self.semaphore.acquire(1).await.unwrap();
        self.tickv.get_key(hash(key), buf)?;
        Ok(())
    }

    pub async fn append_key(&self, key: &[u8], buf: &[u8]) -> Result<()> {
        let _drop = self.semaphore.acquire(1).await.unwrap();
        let res = self.tickv.append_key(hash(key), buf);
        if let Err(e) = res {
            if e == ErrorCode::UnsupportedVersion {
                log::error!(
                    "Unsupported version while appending flash key... Wiping NVS partition!"
                );

                let mut flash = esp_storage::FlashStorage::new(unsafe {
                    self.flash_peripheral.clone_unchecked()
                });
                let mut written = 0;

                while written < self.size {
                    let chunk = [0; 1024];
                    let chunk_size = (self.size - written).min(1024);

                    _ = flash.write((self.offset + written) as u32, &chunk[..chunk_size]);
                    written += chunk_size;
                }

                self.tickv.initialise(hash(tickv::MAIN_KEY))?;
                self.tickv.append_key(hash(key), buf)?;
            }
        }

        Ok(())
    }

    pub async fn invalidate_key(&self, key: &[u8]) -> Result<()> {
        let _drop = self.semaphore.acquire(1).await.unwrap();
        self.tickv.invalidate_key(hash(key))?;
        Ok(())
    }

    /// # Safety
    ///
    /// This doesn't check for semaphore!
    pub unsafe fn get_key_unchecked(&self, key: &[u8], buf: &mut [u8]) -> Result<()> {
        self.tickv.get_key(hash(key), buf)?;
        Ok(())
    }

    /// # Safety
    ///
    /// This doesn't check for semaphore!
    pub unsafe fn append_key_unckeched(&self, key: &[u8], buf: &[u8]) -> Result<()> {
        let res = self.tickv.append_key(hash(key), buf);
        if let Err(e) = res {
            if e == ErrorCode::UnsupportedVersion {
                log::error!(
                    "Unsupported version while appending flash key... Wiping NVS partition!"
                );

                let mut flash = esp_storage::FlashStorage::new(unsafe {
                    self.flash_peripheral.clone_unchecked()
                });
                let mut written = 0;

                while written < self.size {
                    let chunk = [0; 1024];
                    let chunk_size = (self.size - written).min(1024);

                    _ = flash.write((self.offset + written) as u32, &chunk[..chunk_size]);
                    written += chunk_size;
                }

                self.tickv.initialise(hash(tickv::MAIN_KEY))?;
                self.tickv.append_key(hash(key), buf)?;
            }
        }

        Ok(())
    }

    /// # Safety
    ///
    /// This doesn't check for semaphore!
    pub unsafe fn invalidate_key_unchecked(&self, key: &[u8]) -> Result<()> {
        self.tickv.invalidate_key(hash(key))?;
        Ok(())
    }
}

impl Drop for Nvs {
    fn drop(&mut self) {
        NVS_INSTANCES.fetch_sub(1, core::sync::atomic::Ordering::Relaxed);
    }
}

impl Clone for Nvs {
    fn clone(&self) -> Self {
        NVS_INSTANCES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

        Self {
            flash_peripheral: unsafe { self.flash_peripheral.clone_unchecked() },
            tickv: self.tickv.clone(),
            semaphore: self.semaphore.clone(),
            offset: self.offset,
            size: self.size,
        }
    }
}

pub struct NvsFlash {
    flash_offset: u32,
    flash: Mutex<CriticalSectionRawMutex, esp_storage::FlashStorage<'static>>,
}

impl NvsFlash {
    pub fn new(flash_offset: usize, flash: esp_hal::peripherals::FLASH<'static>) -> Self {
        Self {
            flash_offset: flash_offset as u32,
            flash: Mutex::new(esp_storage::FlashStorage::new(flash)),
        }
    }
}

impl FlashController<1024> for NvsFlash {
    fn read_region(
        &self,
        region_number: usize,
        buf: &mut [u8; 1024],
    ) -> core::result::Result<(), tickv::ErrorCode> {
        if let Ok(mut flash) = self.flash.try_lock() {
            let offset = region_number * 1024;
            flash
                .read(self.flash_offset + offset as u32, buf)
                .map_err(|_| tickv::ErrorCode::ReadFail)
        } else {
            Err(tickv::ErrorCode::ReadFail)
        }
    }

    fn write(&self, address: usize, buf: &[u8]) -> core::result::Result<(), tickv::ErrorCode> {
        if let Ok(mut flash) = self.flash.try_lock() {
            flash
                .write(self.flash_offset + address as u32, buf)
                .map_err(|_| tickv::ErrorCode::WriteFail)
        } else {
            Err(tickv::ErrorCode::WriteFail)
        }
    }

    fn erase_region(&self, region_number: usize) -> core::result::Result<(), tickv::ErrorCode> {
        if let Ok(mut flash) = self.flash.try_lock() {
            flash
                .write(
                    self.flash_offset + (region_number as u32 * 1024),
                    &[0xFF; 1024],
                )
                .map_err(|_| tickv::ErrorCode::EraseFail)
        } else {
            Err(tickv::ErrorCode::EraseFail)
        }
    }
}

pub fn hash(buf: &[u8]) -> u64 {
    let mut tmp = 0;
    for b in buf {
        tmp ^= *b as u64;
        tmp <<= 1;
    }

    tmp
}
