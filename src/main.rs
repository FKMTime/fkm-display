#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use crate::nvs::Nvs;
use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use log::info;
use rand_core::OsRng;
use trouble_host::prelude::*;
extern crate alloc;

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
    battery_service: BatteryService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
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

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    run(ble_controller, &nvs).await;

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

pub async fn run<C>(controller: C, nvs: &Nvs)
where
    C: Controller,
{
    let address: Address = Address::random(esp_hal::efuse::Efuse::mac_address());
    info!("Our address = {}", address);

    let mut resources: HostResources<DefaultPacketPool, 1, 3> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut OsRng);

    let mut bond_stored = if let Some(bond_info) = load_bonding_info(nvs).await {
        info!("Loaded bond information");
        stack.add_bond_information(bond_info).unwrap();
        true
    } else {
        info!("No bond information found");
        false
    };

    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = embassy_futures::join::join(ble_task(runner), async {
        loop {
            match advertise("Trouble Example", &mut peripheral, &server).await {
                Ok(conn) => {
                    // Allow bondable if no bond is stored.
                    conn.raw().set_bondable(!bond_stored).unwrap();
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&nvs, &server, &conn, &mut bond_stored);
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
    bond_stored: &mut bool,
) -> core::result::Result<(), Error> {
    let level = server.battery_service.level;
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
                    *bond_stored = true;
                    info!("Bond information stored");
                }
            }
            GattConnectionEvent::PairingFailed(err) => {
                log::error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Gatt { event } => {
                let result = match &event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        }
                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == level.handle {
                            info!(
                                "[gatt] Write Event to Level Characteristic: {:?}",
                                event.data()
                            );
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
    Ok(())
}

async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    loop {
        tick = tick.wrapping_add(1);
        info!("[custom_task] notifying connection of tick {}", tick);
        if level.notify(conn, &tick).await.is_err() {
            info!("[custom_task] error notifying connection");
            break;
        };
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
