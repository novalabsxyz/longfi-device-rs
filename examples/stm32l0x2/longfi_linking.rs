#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]


use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::{Floating, Input, Output, PushPull};
use core::ffi;
use embedded_hal::spi::FullDuplex;
use stm32l0xx_hal::pac::SPI1;
use stm32l0xx_hal as hal;
use nb::block;
use embedded_hal::digital::v2::OutputPin;


#[repr(C, align(4))]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
pub struct SpiInstance {
    Instance: *mut ffi::c_void,
}

#[repr(C, align(4))]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
pub struct Spi_s {
    Spi: SpiInstance,
    Nss: Gpio_t,
}

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
pub type Spi_t = Spi_s;

#[no_mangle]
pub extern "C" fn SpiInOut(s: &mut Spi_t, outData: u16) -> u16 {
    let spi: &mut hal::spi::Spi<
        SPI1,
        (
            PA3<Input<Floating>>,
            PA6<Input<Floating>>,
            PA7<Input<Floating>>,
        ),
    > = unsafe {
        &mut *(s.Spi.Instance
            as *mut hal::spi::Spi<
                SPI1,
                (
                    PA3<Input<Floating>>,
                    PA6<Input<Floating>>,
                    PA7<Input<Floating>>,
                ),
            >)
    };

    spi.send(outData as u8).unwrap();
    let inData = block!(spi.read()).unwrap();

    inData as u16
}

#[allow(non_camel_case_types)]
#[allow(non_snake_case)]
type Gpio_t = *mut ffi::c_void;

#[repr(C)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum PinNames {
    MCU_PINS,
    IOE_PINS,
    RADIO_RESET,
}

#[repr(C)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum PinModes {
    PIN_INPUT = 0,
    PIN_OUTPUT,
    PIN_ALTERNATE_FCT,
    PIN_ANALOGIC,
}

#[repr(C)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum PinTypes {
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN,
}

#[repr(C)]
#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum PinConfigs {
    PIN_PUSH_PULL = 0,
    PIN_OPEN_DRAIN,
}

#[no_mangle]
pub extern "C" fn GpioInit(
    obj: Gpio_t,
    _pin: PinNames,
    _mode: PinModes,
    _config: PinConfigs,
    _pin_type: PinTypes,
    val: u32,
) {
    let mut gpio: &mut stm32l0xx_hal::gpio::gpioc::PC0<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioc::PC0<Output<PushPull>>) };

    if val==0 {
        gpio.set_low();
    } else {
        gpio.set_high();
    }
}

#[no_mangle]
pub extern "C" fn GpioWrite(obj: Gpio_t, val: u8) {
    let gpio: &mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>) };

    if val==0 {
        gpio.set_low().unwrap();
    } else {
        gpio.set_high().unwrap();
    }
}

#[repr(C, align(4))]
pub struct TimerEvent_s {
    IsRunning: bool,
}

#[allow(non_camel_case_types)]
type TimerEvent_t = TimerEvent_s;

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerInit(_obj: &TimerEvent_t, cb: Option<extern "C" fn()>) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerStart(_obj: &TimerEvent_t) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerStop(_obj: &TimerEvent_t) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerReset(_obj: &TimerEvent_t) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerSetValue(_obj: &TimerEvent_t, value: u32) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerGetCurrentTime() {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerGetElapsedTime(_saved_time: &TimerEvent_t) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerGetFutureTime(_event_in_future: &TimerEvent_t) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn TimerLowPowerHandler() {}

#[allow(non_camel_case_types)]
type irq_ptr = extern "C" fn();

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn SX1276GetPaSelect(_channel: u32) -> u8 {
    0
}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn DelayMs(ms: u32) {
}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn SX1276SetAntSwLowPower(status: bool) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn SX1276SetAntSw(rxTx: u8) {}

#[no_mangle]
#[allow(dead_code)]
pub extern "C" fn assert_param(expr: bool) {}
