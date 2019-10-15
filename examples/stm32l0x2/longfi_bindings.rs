use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use longfi_device::{AntPinsMode, Spi};
use nb::block;
use stm32l0xx_hal as hal;
use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::gpiob::*;
use stm32l0xx_hal::gpio::{Floating, Input, Output, PushPull};
use stm32l0xx_hal::pac::SPI2;

static mut ANT_EN: Option<stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>> = None;

pub fn set_ant_en(pin: stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>) {
    unsafe {
        ANT_EN = Some(pin);
    }
}

pub extern "C" fn set_antenna_pins(mode: AntPinsMode, power: u8) {
    unsafe {
        if let Some(ant_en) = &mut ANT_EN {
            match mode {
                AntPinsMode::AntModeTx | AntPinsMode::AntModeRx => {
                    ant_en.set_high();
                }
                AntPinsMode::AntModeSleep => {
                    ant_en.set_low();
                }
                _ => (),
            }
        }
    }
}

static mut EN_TCXO: Option<stm32l0xx_hal::gpio::gpioa::PA8<Output<PushPull>>> = None;
pub fn set_tcxo_pins(pin: stm32l0xx_hal::gpio::gpioa::PA8<Output<PushPull>>) {
    unsafe {
        EN_TCXO = Some(pin);
    }
}

#[no_mangle]
pub extern "C" fn set_tcxo(value: bool) -> u8 {
    unsafe {
        if let Some(pin) = &mut EN_TCXO {
            if value {
                pin.set_high().unwrap();
            } else {
                pin.set_high().unwrap();
            }
        }
    }
    6
}

static mut RADIO_BUSY: Option<stm32l0xx_hal::gpio::gpioc::PC2<Input<Floating>>> = None;
pub fn set_is_busy_pin(pin: stm32l0xx_hal::gpio::gpioc::PC2<Input<Floating>>) {
    unsafe {
        RADIO_BUSY = Some(pin);
    }
}

#[no_mangle]
pub extern "C" fn busy_pin_status() -> bool {
    unsafe {
        if let Some(pin) = &mut RADIO_BUSY {
            return pin.is_high().unwrap();
        }
        true
    }
}

#[no_mangle]
pub extern "C" fn spi_in_out(s: *mut Spi, out_data: u8) -> u8 {
    let spi: &mut hal::spi::Spi<
        SPI2,
        (
            PB13<Input<Floating>>,
            PB14<Input<Floating>>,
            PB15<Input<Floating>>,
        ),
    > = unsafe {
        &mut *((*s).Spi.Instance
            as *mut hal::spi::Spi<
                SPI2,
                (
                    PB13<Input<Floating>>,
                    PB14<Input<Floating>>,
                    PB15<Input<Floating>>,
                ),
            >)
    };

    spi.send(out_data).unwrap();
    let in_data = block!(spi.read()).unwrap();

    in_data
}

static mut SPI_NSS: Option<stm32l0xx_hal::gpio::gpiob::PB12<Output<PushPull>>> = None;

pub fn set_spi_nss(pin: stm32l0xx_hal::gpio::gpiob::PB12<Output<PushPull>>) {
    unsafe {
        SPI_NSS = Some(pin);
    }
}

#[no_mangle]
pub extern "C" fn spi_nss(value: bool) {
    unsafe {
        if let Some(pin) = &mut SPI_NSS {
            if value {
                pin.set_high().unwrap();
            } else {
                pin.set_low().unwrap();
            }
        }
    }
}
static mut RESET: Option<stm32l0xx_hal::gpio::gpiob::PB1<Output<PushPull>>> = None;

pub fn set_radio_reset(pin: stm32l0xx_hal::gpio::gpiob::PB1<Output<PushPull>>) {
    unsafe {
        RESET = Some(pin);
    }
}

#[no_mangle]
pub extern "C" fn radio_reset(value: bool) {
    unsafe {
        if let Some(pin) = &mut RESET {
            if value {
                pin.set_low().unwrap();
            } else {
                pin.set_high().unwrap();
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms);
}

#[no_mangle]
pub extern "C" fn get_random_bits(_bits: u8) -> u32 {
    0x1
}
