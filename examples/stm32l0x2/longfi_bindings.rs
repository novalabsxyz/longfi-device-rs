use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use longfi_device::{Gpio, Spi};
use nb::block;
use stm32l0xx_hal as hal;
use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::{Floating, Input, Output, PushPull};
use stm32l0xx_hal::pac::SPI1;

#[no_mangle]
pub extern "C" fn spi_in_out(s: *mut Spi, out_data: u8) -> u8 {
    let spi: &mut hal::spi::Spi<
        SPI1,
        (
            PA3<Input<Floating>>,
            PA6<Input<Floating>>,
            PA7<Input<Floating>>,
        ),
    > = unsafe {
        &mut *((*s).Spi.Instance
            as *mut hal::spi::Spi<
                SPI1,
                (
                    PA3<Input<Floating>>,
                    PA6<Input<Floating>>,
                    PA7<Input<Floating>>,
                ),
            >)
    };

    spi.send(out_data).unwrap();
    let in_data = block!(spi.read()).unwrap();

    in_data
}

#[no_mangle]
pub extern "C" fn gpio_write(obj: *mut Gpio, value: bool) {
    let gpio: &mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>) };

    if value {
        gpio.set_high().unwrap();
    } else {
        gpio.set_low().unwrap();
    }
}

#[no_mangle]
pub extern "C" fn gpio_read(_obj: *mut Gpio) -> bool {
    false
}

#[no_mangle]
pub extern "C" fn delay_ms(_ms: u32) {}

#[no_mangle]
pub extern "C" fn get_random_bits(_bits: u8) -> u32 {
    0x1
}
