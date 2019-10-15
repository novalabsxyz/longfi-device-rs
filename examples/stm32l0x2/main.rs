#![cfg_attr(not(test), no_std)]
#![no_main]

mod longfi_bindings;

extern crate nb;
extern crate panic_halt;

use core::fmt::Write;
use embedded_hal::digital::v2::OutputPin;
use hal::serial::USART1 as DebugUsart;
use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc, serial, spi, syscfg};
use longfi_device;
use longfi_device::LongFi;
use longfi_device::{ClientEvent, Config, RfEvent};
use stm32l0xx_hal as hal;

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

        let sck = gpiob.pb13;
        let miso = gpiob.pb14;
        let mosi = gpiob.pb15;
        let nss = gpiob.pb12.into_push_pull_output();
        longfi_bindings::set_spi_nss(nss);

        // Initialise the SPI peripheral.
        let mut _spi = device
            .SPI2
            .spi((sck, miso, mosi), spi::MODE_0, 1_000_000.hz(), &mut rcc);

        let reset = gpiob.pb1.into_push_pull_output();
        longfi_bindings::set_radio_reset(reset);

        let busy = gpioc.pc2.into_floating_input();
        longfi_bindings::set_is_busy_pin(busy);

        let ant_en = gpioa.pa15.into_push_pull_output();
        longfi_bindings::set_ant_en(ant_en);

        static mut BINDINGS: longfi_device::BoardBindings = longfi_device::BoardBindings {
            reset: Some(longfi_bindings::radio_reset),
            spi_in_out: Some(longfi_bindings::spi_in_out),
            spi_nss: Some(longfi_bindings::spi_nss),
            delay_ms: Some(longfi_bindings::delay_ms),
            get_random_bits: Some(longfi_bindings::get_random_bits),
            set_antenna_pins: Some(longfi_bindings::set_antenna_pins),
            set_board_tcxo: None,
            busy_pin_status: Some(longfi_bindings::busy_pin_status),
        };

        let rf_config = Config {
            oui: 1234,
            device_id: 5678,
            auth_mode: longfi_device::AuthMode::PresharedKey128,
        };

        let mut longfi_radio =
            unsafe { LongFi::new(&mut BINDINGS, rf_config, Some(get_preshared_key)).unwrap() };

        longfi_radio.set_buffer(resources.BUFFER);

        // let value = unsafe { longfi_sys::SX126xReadRegister(0x06BC) };

        //let packet: [u8; 5] = [0xDE, 0xAD, 0xBE, 0xEF, 0];
        //longfi_radio.send(&packet);
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
        write!(resources.DEBUG_UART, "Event!\r\n").unwrap();

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
        let packet: [u8; 5] = [
            0xDE,
            0xAD,
            0xBE,
            0xEF,
            *resources.COUNT,
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
