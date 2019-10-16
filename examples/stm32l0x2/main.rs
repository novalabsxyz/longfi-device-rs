#![cfg_attr(not(test), no_std)]
#![no_main]

#[cfg(not(any(feature = "helium_feather", feature = "b_l072z_lrwan1")))]
panic!{"Must do \"--features\" for one of the support boards while building the example"}

extern crate nb;
extern crate panic_halt;


use stm32l0xx_hal as hal;
use hal::serial::USART1 as DebugUsart;
use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc, serial, syscfg};

use longfi_device;
use longfi_device::{LongFi, RadioType, ClientEvent, Config, RfEvent};

use core::fmt::Write;

#[cfg(feature = "helium_feather")]
use helium_tracker_feather as board;
#[cfg(feature = "b_l072z_lrwan1")]
use b_l072z_lrwan1 as board;

static mut PRESHARED_KEY: [u8; 16] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];

pub extern "C" fn get_preshared_key() -> *mut u8 {
    unsafe { &mut PRESHARED_KEY[0] as *mut u8 }
}

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB5<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();
    static mut BUTTON: gpiob::PB2<Input<PullUp>> = ();
    static mut SX126X_DIO1: gpiob::PB0<Input<Floating>> = ();
    static mut DEBUG_UART: serial::Tx<DebugUsart> = ();
    static mut UART_RX: serial::Rx<DebugUsart> = ();
    static mut BUFFER: [u8; 512] = [0; 512];
    static mut COUNT: u8 = 0;
    static mut LONGFI: LongFi = ();

    #[init(spawn = [send_ping], resources = [BUFFER])]
    fn init() -> init::LateResources {
        // Configure the clock.
        let mut rcc = device.RCC.freeze(rcc::Config::hsi16());
        let mut syscfg = syscfg::SYSCFG::new(device.SYSCFG_COMP, &mut rcc);

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        let tx_pin = gpioa.pa9;
        let rx_pin = gpioa.pa10;

        // Configure the serial peripheral.
        let mut serial = device
            .USART1
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        serial.listen(serial::Event::Rxne);

        let (mut tx, mut rx) = serial.split();

        write!(tx, "SX1262 test\r\n").unwrap();

        // Configure PB5 as output.
        let led = gpiob.pb5.into_push_pull_output();

        let exti = device.EXTI;

        // Configure PB2 as input.
        let button = gpiob.pb2.into_pull_up_input();
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(&mut syscfg, button.port, button.i, TriggerEdge::Falling);

        // // Configure PB4 as input.
        let sx126x_dio1 = gpiob.pb0.into_floating_input();
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(
            &mut syscfg,
            sx126x_dio1.port,
            sx126x_dio1.i,
            TriggerEdge::Rising,
        );

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
                gpioc.pc2
            );
        }

        // #[cfg(feature = "b_l072z_lrwan1")]
        // unsafe {
        //     BINDINGS.init(
        //         device.SPI1,
        //         &mut rcc,
        //         gpiob.pb3,
        //         gpioa.pa6,
        //         gpioa.pa7,
        //         gpioa.pa15,
        //         gpioc.pc0,
        //         gpioa.pa1,
        //         gpioc.pc2,
        //         gpioc.pc1
        //     );
        // }
        let rf_config = Config {
            oui: 1234,
            device_id: 5678,
            auth_mode: longfi_device::AuthMode::PresharedKey128,
        };

        let radio = RadioType::SX1276;

        let mut longfi_radio =
            unsafe { LongFi::new(radio, &mut BINDINGS.bindings, rf_config, Some(get_preshared_key)).unwrap() };

        longfi_radio.set_buffer(resources.BUFFER);

        write!(tx, "Going to main loop\r\n").unwrap();

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
            BUTTON: button,
            SX126X_DIO1: sx126x_dio1,
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

    #[interrupt(priority = 1, resources = [LED, INT, BUTTON], spawn = [send_ping])]
    fn EXTI2_3() {
        static mut STATE: bool = false;
        // Clear the interrupt flag.
        resources.INT.clear_irq(resources.BUTTON.i);
        if *STATE {
            resources.LED.set_low().unwrap();
            *STATE = false;
        } else {
            resources.LED.set_high().unwrap();
            *STATE = true;
        }
        spawn.send_ping().unwrap();
    }

    #[interrupt(priority=1, resources = [UART_RX], spawn = [send_ping])] // = 1, )]
    fn USART1() {
        let rx = resources.UART_RX;
        rx.read().unwrap();
        spawn.send_ping().unwrap();
    }

    #[interrupt(priority = 1, resources = [SX126X_DIO1, INT], spawn = [radio_event])]
    fn EXTI0_1() {
        resources.INT.clear_irq(resources.SX126X_DIO1.i);
        spawn.radio_event(RfEvent::DIO0).unwrap();
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }
};
