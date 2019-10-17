// To use example, press any key in serial terminal
// Packet will send and "Transmit Done!" will print when radio is done sending packet

#![cfg_attr(not(test), no_std)]
#![no_main]

#[cfg(not(any(
    feature = "helium_feather",
    feature = "b_l072z_lrwan1",
    feature = "catena_4610"
)))]
compile_error!{"Must do \"--features\" for one of the support boards while building the example"}

extern crate nb;
extern crate panic_halt;

use hal::{gpio::*, pac, prelude::*, rcc, serial, syscfg};
use stm32l0xx_hal as hal;

use longfi_device;
use longfi_device::{ClientEvent, Config, LongFi, RadioType, RfEvent};

use core::fmt::Write;

#[cfg(feature = "b_l072z_lrwan1")]
use b_l072z_lrwan1 as board;
#[cfg(feature = "catena_4610")]
use catena_4610 as board;
#[cfg(feature = "helium_feather")]
use helium_tracker_feather as board;

static mut PRESHARED_KEY: [u8; 16] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];

pub extern "C" fn get_preshared_key() -> *mut u8 {
    unsafe { &mut PRESHARED_KEY[0] as *mut u8 }
}

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut INT: pac::EXTI = ();
    static mut RADIO_IRQ: board::RadioIRQ = ();
    static mut DEBUG_UART: serial::Tx<board::DebugUsart> = ();
    static mut UART_RX: serial::Rx<board::DebugUsart> = ();
    static mut BUFFER: [u8; 512] = [0; 512];
    static mut COUNT: u8 = 0;
    static mut LONGFI: LongFi = ();

    #[init(spawn = [send_ping], resources = [BUFFER])]
    fn init() -> init::LateResources {
        let mut rcc = device.RCC.freeze(rcc::Config::hsi16());
        let mut syscfg = syscfg::SYSCFG::new(device.SYSCFG_COMP, &mut rcc);

        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        #[cfg(feature = "b_l072z_lrwan1")]
        let (tx_pin, rx_pin, serial_peripheral) = (gpioa.pa2, gpioa.pa3, device.USART2);
        #[cfg(any(feature = "helium_feather", feature = "catena_4610"))]
        let (tx_pin, rx_pin, serial_peripheral) = (gpioa.pa9, gpioa.pa10, device.USART1);

        let mut serial = serial_peripheral
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        // listen for incoming bytes which will trigger transmits
        serial.listen(serial::Event::Rxne);
        let (mut tx, mut rx) = serial.split();

        write!(tx, "LongFi Device Test\r\n").unwrap();

        let mut exti = device.EXTI;
        #[cfg(any(feature = "b_l072z_lrwan1", feature = "catena_4610"))]
        let radio_irq = gpiob.pb4;
        #[cfg(feature = "helium_feather")]
        let radio_irq = gpiob.pb0;

        let radio_irq = board::initialize_radio_irq(radio_irq, &mut syscfg, &mut exti);

        static mut BINDINGS: board::LongFiBindings = board::LongFiBindings::new();

        #[cfg(feature = "helium_feather")]
        unsafe {
            BINDINGS.init(
                device.SPI2,
                &mut rcc,
                gpiob.pb13,
                gpiob.pb14,
                gpiob.pb15,
                gpiob.pb12,
                gpiob.pb1,
                gpioa.pa15,
                gpioc.pc2,
            );
        }

        #[cfg(feature = "b_l072z_lrwan1")]
        let tcxo_en: Option<board::TcxoEn> = None;
        #[cfg(feature = "catena_4610")]
        let tcxo_en: Option<board::TcxoEn> = Some(gpioa.pa8.into_push_pull_output());

        #[cfg(any(feature = "b_l072z_lrwan1", feature = "catena_4610"))]
        unsafe {
            BINDINGS.init(
                device.SPI1,
                &mut rcc,
                gpiob.pb3,
                gpioa.pa6,
                gpioa.pa7,
                gpioa.pa15,
                gpioc.pc0,
                gpioa.pa1,
                gpioc.pc2,
                gpioc.pc1,
                tcxo_en,
            );
        }

        let rf_config = Config {
            oui: 1234,
            device_id: 5678,
            auth_mode: longfi_device::AuthMode::PresharedKey128,
        };

        #[cfg(feature = "helium_feather")]
        let radio = RadioType::Sx1262;

        #[cfg(any(feature = "b_l072z_lrwan1", feature = "catena_4610"))]
        let radio = RadioType::Sx1276;

        let mut longfi_radio = unsafe {
            LongFi::new(
                radio,
                &mut BINDINGS.bindings,
                rf_config,
                Some(get_preshared_key),
            )
            .unwrap()
        };

        longfi_radio.set_buffer(resources.BUFFER);

        write!(tx, "Going to main loop\r\n").unwrap();

        // Return the initialised resources.
        init::LateResources {
            INT: exti,
            RADIO_IRQ: radio_irq,
            DEBUG_UART: tx,
            UART_RX: rx,
            LONGFI: longfi_radio,
        }
    }

    #[task(capacity = 4, priority = 2, resources = [DEBUG_UART, BUFFER, LONGFI])]
    fn radio_event(event: RfEvent) {
        let mut longfi_radio = resources.LONGFI;
        let client_event = longfi_radio.handle_event(event);

        match client_event {
            ClientEvent::ClientEvent_TxDone => {
                write!(resources.DEBUG_UART, "Transmit Done!\r\n").unwrap();
            }
            ClientEvent::ClientEvent_Rx => {
                // get receive buffer
                let rx_packet = longfi_radio.get_rx();
                write!(resources.DEBUG_UART, "Received packet\r\n").unwrap();
                write!(resources.DEBUG_UART, "  Length =  {}\r\n", rx_packet.len).unwrap();
                write!(resources.DEBUG_UART, "  Rssi   = {}\r\n", rx_packet.rssi).unwrap();
                write!(resources.DEBUG_UART, "  Snr    =  {}\r\n", rx_packet.snr).unwrap();
                unsafe {
                    for i in 0..rx_packet.len {
                        write!(
                            resources.DEBUG_UART,
                            "{:X} ",
                            *rx_packet.buf.offset(i as isize)
                        )
                        .unwrap();
                    }
                    write!(resources.DEBUG_UART, "\r\n").unwrap();
                }
                // give buffer back to library
                longfi_radio.set_buffer(resources.BUFFER);
            }
            ClientEvent::ClientEvent_None => {}
        }
    }

    #[task(capacity = 4, priority = 2, resources = [DEBUG_UART, COUNT, LONGFI])]
    fn send_ping() {
        write!(resources.DEBUG_UART, "Sending Ping\r\n").unwrap();
        let packet: [u8; 72] = [
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            *resources.COUNT,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xa1,
            0xa2,
            0xa3,
            0xa4,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            *resources.COUNT,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xa1,
            0xa2,
            0xa3,
            0xa4,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            0xa1,
            0xa2,
            0xa3,
            0xa4,
            0xBE,
            0xEF,
            0xa1,
            0xa2,
            0xa3,
            0xa4,
        ];
        *resources.COUNT += 1;
        resources.LONGFI.send(&packet);
    }

    #[cfg(any(feature = "helium_feather", feature = "catena_4610"))]
    #[interrupt(priority=1, resources = [UART_RX], spawn = [send_ping])]
    fn USART1() {
        let mut rx = resources.UART_RX;
        rx.read().unwrap();
        spawn.send_ping().unwrap();
    }

    #[cfg(feature = "b_l072z_lrwan1")]
    #[interrupt(priority=1, resources = [UART_RX], spawn = [send_ping])]
    fn USART2() {
        let mut rx = resources.UART_RX;
        rx.read().unwrap();
        spawn.send_ping().unwrap();
    }

    #[cfg(feature = "helium_feather")]
    #[interrupt(priority = 1, resources = [RADIO_IRQ, INT], spawn = [radio_event])]
    fn EXTI0_1() {
        resources.INT.clear_irq(resources.RADIO_IRQ.i);
        spawn.radio_event(RfEvent::DIO0).unwrap();
    }

    #[cfg(any(feature = "helium_feather", feature = "catena_4610"))]
    #[interrupt(priority = 1, resources = [RADIO_IRQ, INT], spawn = [radio_event])]
    fn EXTI4_15() {
        resources.INT.clear_irq(resources.RADIO_IRQ.i);
        spawn.radio_event(RfEvent::DIO0).unwrap();
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }
};
