#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use crate::lcd_abstract::{LcdAbstract, PrintAlign};
use crate::nvs::Nvs;
use adv_shift_registers::wrappers::{ShifterPin, ShifterValue};
use ag_lcd_async::LcdDisplay;
use alloc::string::{String, ToString as _};
use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use log::info;
use rand_core::OsRng;
use trouble_host::prelude::*;
extern crate alloc;

mod lcd_abstract;
mod nvs;

pub fn custom_rng(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    for chunk in buf.chunks_mut(4) {
        let random_u32 = unsafe { &*esp_hal::peripherals::RNG::PTR }
            .data()
            .read()
            .bits();

        let len = chunk.len();
        chunk[..].copy_from_slice(&random_u32.to_be_bytes()[..len]);
    }

    Ok(())
}
getrandom::register_custom_getrandom!(custom_rng);

esp_bootloader_esp_idf::esp_app_desc!();

#[gatt_server]
struct Server {
    digits_service: DigitsService,
}

/// Battery service
#[gatt_service(uuid = "a5bad9f2-700a-4c3d-b9e2-e58ad262d40e")]
struct DigitsService {
    #[characteristic(uuid = "a5178cad-e4e0-4598-8053-a4a78b9281e2", write)]
    digits: u64,
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let nvs = nvs::Nvs::new_from_part_table(peripherals.FLASH).unwrap();

    let shifter_data_pin = Output::new(peripherals.GPIO10, Level::Low, Default::default());
    let shifter_latch_pin = Output::new(peripherals.GPIO1, Level::Low, Default::default());
    let shifter_clk_pin = Output::new(peripherals.GPIO21, Level::Low, Default::default());

    let adv_shift_reg = adv_shift_registers::AdvancedShiftRegister::<8, _>::new(
        shifter_data_pin,
        shifter_clk_pin,
        shifter_latch_pin,
        0,
    );
    let adv_shift_reg = alloc::boxed::Box::new(adv_shift_reg);
    let adv_shift_reg = alloc::boxed::Box::leak(adv_shift_reg);

    let mut backlight = adv_shift_reg.get_pin_mut(1, 1, false);
    _ = backlight.set_high();
    let lcd = adv_shift_reg.get_shifter_mut(1);

    let mut lcd = {
        let bl_pin = lcd.get_pin_mut(1, true);
        let rs_pin = lcd.get_pin_mut(2, true);
        let en_pin = lcd.get_pin_mut(3, true);
        let d4_pin = lcd.get_pin_mut(4, false);
        let d5_pin = lcd.get_pin_mut(5, false);
        let d6_pin = lcd.get_pin_mut(6, false);
        let d7_pin = lcd.get_pin_mut(7, false);
        LcdDisplay::new(rs_pin, en_pin, Delay)
            .with_display(ag_lcd_async::Display::On)
            .with_blink(ag_lcd_async::Blink::Off)
            .with_cursor(ag_lcd_async::Cursor::Off)
            .with_size(ag_lcd_async::Size::Dots5x8)
            .with_cols(16)
            .with_lines(ag_lcd_async::Lines::TwoLines)
            .with_half_bus(d4_pin, d5_pin, d6_pin, d7_pin)
            .with_backlight(bl_pin)
            .build()
            .await
    };

    lcd.clear().await;
    lcd.backlight_on();

    let lcd_driver: LcdAbstract<80, 16, 2, 3> = LcdAbstract::new();

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    run(ble_controller, &nvs, lcd_driver, lcd).await;

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }
}

async fn store_bonding_info(nvs: &Nvs, info: &BondInformation) {
    let mut buf = [0; 32];
    _ = nvs.invalidate_key(b"BONDING_INFO").await;

    buf[..6].copy_from_slice(info.identity.bd_addr.raw());
    buf[6..22].copy_from_slice(info.ltk.to_le_bytes().as_slice());
    log::info!(
        "store {:?} {:?} {:?}",
        info.identity.bd_addr,
        info.ltk,
        info.security_level
    );
    buf[22] = match info.security_level {
        SecurityLevel::NoEncryption => 0,
        SecurityLevel::Encrypted => 1,
        SecurityLevel::EncryptedAuthenticated => 2,
    };

    nvs.append_key(b"BOUNDING_KEY", &buf).await.unwrap();
}

async fn load_bonding_info(nvs: &Nvs) -> Option<BondInformation> {
    let mut buf = [0; 32];
    let res = nvs.get_key(b"BOUNDING_KEY", &mut buf).await;
    if res.is_err() {
        return None;
    }

    let bd_addr = BdAddr::new(buf[..6].try_into().unwrap());
    let security_level = match buf[22] {
        0 => SecurityLevel::NoEncryption,
        1 => SecurityLevel::Encrypted,
        2 => SecurityLevel::EncryptedAuthenticated,
        _ => return None,
    };
    let ltk = LongTermKey::from_le_bytes(buf[6..22].try_into().unwrap());

    log::info!("load {:?} {:?} {:?}", bd_addr, ltk, security_level);
    return Some(BondInformation {
        identity: Identity { bd_addr, irk: None },
        security_level,
        is_bonded: true,
        ltk,
    });
}

pub async fn run<C>(
    controller: C,
    nvs: &Nvs,
    mut lcd: LcdAbstract<80, 16, 2, 3>,
    mut lcd_raw: LcdDisplay<ShifterPin, Delay>,
) where
    C: Controller,
{
    let address: Address = Address::random(esp_hal::efuse::Efuse::mac_address());
    info!("Our address = {}", address);

    let mut resources: HostResources<DefaultPacketPool, 1, 3> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut OsRng);

    let mut bond_info = if let Some(bond_info) = load_bonding_info(nvs).await {
        info!("Loaded bond information");
        stack.add_bond_information(bond_info.clone()).unwrap();
        Some(bond_info)
    } else {
        info!("No bond information found");
        _ = lcd.print(0, "Connect To FKM", PrintAlign::Center, true);
        lcd.display_on_lcd(&mut lcd_raw).await;
        None
    };

    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "FkmDisplay",
        appearance: &appearance::display_equipment::GENERIC_DISPLAY_EQUIPMENT,
    }))
    .unwrap();

    let _ = embassy_futures::join::join(ble_task(runner), async {
        let hostname = alloc::format!("FKMD-{:X}", get_efuse_u32());
        loop {
            match advertise(
                &hostname,
                &mut peripheral,
                &server,
                &bond_info,
                &mut lcd,
                &mut lcd_raw,
            )
            .await
            {
                Ok(conn) => {
                    if let Some(bond) = &bond_info {
                        if conn.raw().peer_address() != bond.identity.bd_addr.into() {
                            info!("Rejecting connection from unknown device");
                            conn.raw().disconnect();
                            continue;
                        }
                    }

                    // Allow bondable if no bond is stored.
                    _ = conn.raw().set_bondable(bond_info.is_none());
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(
                        &nvs,
                        &server,
                        &conn,
                        &mut bond_info,
                        &mut lcd,
                        &mut lcd_raw,
                    );
                    let b = custom_task(&server, &conn, &stack);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    embassy_futures::select::select(a, b).await;
                }
                Err(e) => {
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

async fn gatt_events_task(
    nvs: &Nvs,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    bond_info: &mut Option<BondInformation>,
    lcd: &mut LcdAbstract<80, 16, 2, 3>,
    lcd_raw: &mut LcdDisplay<ShifterPin, Delay>,
) -> core::result::Result<(), Error> {
    let digits = server.digits_service.digits;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::PairingComplete {
                security_level,
                bond,
            } => {
                info!("[gatt] pairing complete: {:?}", security_level);
                if let Some(bond) = bond {
                    store_bonding_info(nvs, &bond).await;
                    *bond_info = Some(bond);
                    info!("Bond information stored");
                    _ = lcd.print(0, "Connected", PrintAlign::Center, true);
                    lcd.display_on_lcd(lcd_raw).await;
                }
            }
            GattConnectionEvent::PairingFailed(err) => {
                log::error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Gatt { event } => {
                let result = match &event {
                    GattEvent::Read(_event) => {
                        /*
                        if event.handle() == digits.handle {
                            let value = server.get(&digits);
                            info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        }
                        */

                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == digits.handle {
                            let ms: u64 = u64::from_be_bytes(event.data().try_into().unwrap());
                            let time_str = ms_to_time_str(ms);
                            log::info!("Display Time: {time_str}");
                            _ = lcd.print(0, &time_str, PrintAlign::Center, true);
                            lcd.display_on_lcd(lcd_raw).await;
                        }
                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    _ => None,
                };

                let reply_result = if let Some(code) = result {
                    event.reject(code)
                } else {
                    event.accept()
                };
                match reply_result {
                    Ok(reply) => reply.send().await,
                    Err(e) => log::warn!("[gatt] error sending response: {:?}", e),
                }
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    _ = lcd.print(0, "Disconnected", PrintAlign::Center, true);
    lcd.display_on_lcd(lcd_raw).await;
    Ok(())
}

async fn advertise<'values, 'server, C: Controller>(
    name: &str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
    bond_info: &Option<BondInformation>,
    lcd: &mut LcdAbstract<80, 16, 2, 3>,
    lcd_raw: &mut LcdDisplay<ShifterPin, Delay>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let advertiser = if let Some(bond) = bond_info {
        info!("[adv] advertising directed");
        peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableNonscannableDirectedHighDuty {
                    peer: Address::random(bond.identity.bd_addr.raw().try_into().unwrap()),
                },
            )
            .await?
    } else {
        let mut advertiser_data = [0; 31];
        let len = AdStructure::encode_slice(
            &[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
                AdStructure::CompleteLocalName(name.as_bytes()),
            ],
            &mut advertiser_data[..],
        )?;
        info!("[adv] advertising undirected");
        peripheral
            .advertise(
                &Default::default(),
                Advertisement::ConnectableScannableUndirected {
                    adv_data: &advertiser_data[..len],
                    scan_data: &[],
                },
            )
            .await?
    };

    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    _ = lcd.print(0, "Connected", PrintAlign::Center, true);
    lcd.display_on_lcd(lcd_raw).await;
    Ok(conn)
}

async fn custom_task<C: Controller, P: PacketPool>(
    _server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    loop {
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            info!("[custom_task] error getting RSSI");
            break;
        };

        Timer::after_secs(2).await;
    }
}

pub fn get_efuse_mac() -> u64 {
    esp_hal::efuse::Efuse::mac_address()
        .iter()
        .fold(0u64, |acc, &x| (acc << 8) + x as u64)
}

pub fn get_efuse_u32() -> u32 {
    let mut efuse = get_efuse_mac();
    efuse = (!efuse).wrapping_add(efuse << 18);
    efuse = efuse ^ (efuse >> 31);
    efuse = efuse.wrapping_mul(21);
    efuse = efuse ^ (efuse >> 11);
    efuse = efuse.wrapping_add(efuse << 6);
    efuse = efuse ^ (efuse >> 22);

    let mac = efuse & 0x000000007FFFFFFF;
    mac as u32
}

pub fn ms_to_time_str(ms: u64) -> heapless::String<12> {
    let minutes: u8 = (ms / 60000) as u8;
    let seconds: u8 = ((ms % 60000) / 1000) as u8;
    let ms: u16 = (ms % 1000) as u16;

    let mut time_str = heapless::String::<12>::new();
    if minutes > 0 {
        _ = time_str.push_str(&alloc::format!("{minutes}:{seconds:02}.{ms:03}"));
    } else {
        _ = time_str.push_str(&alloc::format!("{seconds:01}.{ms:03}"));
    }

    time_str
}
