#![no_std]
#![no_main]

mod longfi_linking;
/*
/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PC_0

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PB_3

#define RADIO_NSS                                   PA_15

#define RADIO_DIO_0                                 PB_4
#define RADIO_DIO_1                                 PB_1
#define RADIO_DIO_2                                 PB_0
#define RADIO_DIO_3                                 PC_13
#define RADIO_DIO_4                                 PA_5
#define RADIO_DIO_5                                 PA_4

#define RADIO_TCXO_POWER                            PA_12

#define RADIO_ANT_SWITCH_RX                         PA_1
#define RADIO_ANT_SWITCH_TX_BOOST                   PC_1
#define RADIO_ANT_SWITCH_TX_RFO                     PC_2
*/

extern crate nb;
extern crate panic_halt;

use core::fmt::Write;
use hal::serial::USART2;
use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, serial, spi};
use longfi_device;
use longfi_device::LongFi;
use longfi_device::{ClientEvent, QualityOfService, RfConfig, RfEvent};
use stm32l0xx_hal as hal;

use embedded_hal::digital::v2::OutputPin;

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB5<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();
    static mut BUTTON: gpiob::PB2<Input<PullUp>> = ();
    static mut SX1276_DIO0: gpiob::PB4<Input<PullUp>> = ();
    static mut DEBUG_UART: serial::Tx<USART2> = ();
    static mut BUFFER: [u8; 512] = [0; 512];
    static mut COUNT: u8 = 0;
    static mut LONGFI: LongFi = ();

    #[init(resources = [BUFFER])]
    fn init() -> init::LateResources {
        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;

        // Configure the serial peripheral.
        let serial = device
            .USART2
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        let (mut tx, mut _rx) = serial.split();

        write!(tx, "SX1276 test\r\n").unwrap();

        // Configure PB5 as output.
        let led = gpiob.pb5.into_push_pull_output();

        let exti = device.EXTI;

        // Configure PB2 as input.
        let button = gpiob.pb2.into_pull_up_input();
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            button.port,
            button.i,
            TriggerEdge::Falling,
        );

        // // Configure PB4 as input.
        let sx1276_dio0 = gpiob.pb4.into_pull_up_input();
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            sx1276_dio0.port,
            sx1276_dio0.i,
            TriggerEdge::Rising,
        );

        let sck = gpiob.pb3;
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7;
        let _nss = gpioa.pa15.into_push_pull_output();

        // Initialise the SPI peripheral.
        let mut _spi = device
            .SPI1
            .spi((sck, miso, mosi), spi::MODE_0, 1_000_000.hz(), &mut rcc);

        let mut longfi_radio = LongFi::new();

        // Get the delay provider.
        let mut delay = core.SYST.delay(rcc.clocks);
        let mut reset = gpioc.pc0.into_push_pull_output();

        let mut en_tcxo = gpiob.pb14.into_push_pull_output();

        //#define RADIO_ANT_SWITCH_RX                  STM32L0_GPIO_PIN_PA1
        //#define RADIO_ANT_SWITCH_TX_RFO              STM32L0_GPIO_PIN_PC2
        //#define RADIO_ANT_SWITCH_TX_BOOST            STM32L0_GPIO_PIN_PC1
        let _ant_sw_rx = gpioa.pa1.into_push_pull_output();
        let _ant_sw_tx_rfo = gpioc.pc2.into_push_pull_output();
        let _ant_sw_tx_boost = gpioc.pc1.into_push_pull_output();

        en_tcxo.set_high().unwrap();
        reset.set_low().unwrap();
        delay.delay_ms(1_u16);
        reset.set_high().unwrap();
        delay.delay_ms(6_u16);
        longfi_radio.enable_tcxo();
        reset.set_low().unwrap();
        delay.delay_ms(1_u16);
        reset.set_high().unwrap();

        let config = RfConfig {
            always_on: true,
            qos: QualityOfService::QOS_0,
            network_poll: 200,
            oui: 0x12345678,
            device_id: 0x9abc,
        };

        longfi_radio.initialize(config);
        longfi_radio.set_buffer(resources.BUFFER);

        let packet: [u8; 5] = [0xDE, 0xAD, 0xBE, 0xEF, 0];
        longfi_radio.send(&packet);

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
            BUTTON: button,
            SX1276_DIO0: sx1276_dio0,
            DEBUG_UART: tx,
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
                longfi_radio.set_rx();
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

    #[interrupt(priority = 1, resources = [SX1276_DIO0, INT], spawn = [radio_event])]
    fn EXTI4_15() {
        resources.INT.clear_irq(resources.SX1276_DIO0.i);
        spawn.radio_event(RfEvent::DIO0).unwrap();
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }
};
