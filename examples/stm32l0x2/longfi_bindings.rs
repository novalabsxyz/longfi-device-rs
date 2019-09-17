use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::FullDuplex;
use longfi_device::{
    GpioIrqHandler, Gpio_t, IrqModes, IrqPriorities, PinConfigs, PinModes, PinNames, PinTypes,
    Spi_t,
};
use nb::block;
use stm32l0xx_hal as hal;
use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::{Floating, Input, Output, PushPull};
use stm32l0xx_hal::pac::SPI1;

#[no_mangle]
pub extern "C" fn spi_in_out(s: *mut Spi_t, out_data: u16) -> u16 {
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

    spi.send(out_data as u8).unwrap();
    let in_data = block!(spi.read()).unwrap();

    in_data as u16
}

#[no_mangle]
pub extern "C" fn gpio_init(
    _obj: *mut Gpio_t,
    _pin: PinNames,
    _mode: PinModes,
    _config: PinConfigs,
    _pin_type: PinTypes,
    _value: u32,
) {
}

#[no_mangle]
pub extern "C" fn gpio_write(obj: *mut Gpio_t, value: u32) {
    let gpio: &mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>) };

    if value == 0 {
        gpio.set_low().unwrap();
    } else {
        gpio.set_high().unwrap();
    }
}

#[no_mangle]
pub extern "C" fn gpio_read(_obj: *mut Gpio_t) -> u32 {
    0
}

#[no_mangle]
pub extern "C" fn gpio_set_interrupt(
    _obj: *mut Gpio_t,
    _irq_mode: IrqModes,
    _irq_priority: IrqPriorities,
    _irq_handler: GpioIrqHandler,
) {
}

#[no_mangle]
pub extern "C" fn delay_ms(_ms: u32) {}
