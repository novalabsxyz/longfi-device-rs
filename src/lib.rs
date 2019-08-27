#![cfg_attr(not(test), no_std)]
use longfi_sys;

pub use longfi_sys::BoardBindings_t as BoardBindings;
pub use longfi_sys::ClientEvent;
pub use longfi_sys::LongFi_t;
pub use longfi_sys::QualityOfService;
use longfi_sys::Radio_t;
pub use longfi_sys::RfConfig_t as RfConfig;
pub use longfi_sys::RfEvent;
pub use longfi_sys::RxPacket;

#[cfg(not(test))]
#[macro_use(singleton)]
#[cfg(not(test))]
extern crate cortex_m;

// feature sx1276
static mut SX1276: Option<Radio_t> = None;

pub struct LongFi {
    c_handle: LongFi_t,
}

#[derive(Debug)]
pub enum Error {
    NoRadioPointer,
}

unsafe impl Send for LongFi {}

impl LongFi {
    pub fn new(bindings: &mut BoardBindings, config: RfConfig) -> Result<LongFi, Error> {
        #[cfg(not(test))]
        let _x: &'static mut bool = singleton!(: bool = false).unwrap();

        unsafe {
            SX1276 = Some(longfi_sys::SX1276RadioNew());
            if let Some(radio) = &mut SX1276 {
                let radio_ptr: *mut Radio_t = radio;
                Ok(LongFi {
                    c_handle: longfi_sys::longfi_new_handle(bindings, radio_ptr, config),
                })
            } else {
                Err(Error::NoRadioPointer)
            }
        }
    }

    pub fn initialize(&mut self) {
        unsafe {
            longfi_sys::longfi_init(&mut self.c_handle);
        }
    }

    pub fn set_buffer(&mut self, buffer: &mut [u8]) {
        unsafe {
            longfi_sys::longfi_set_buf(&mut self.c_handle, buffer.as_mut_ptr(), buffer.len());
        }
    }

    //for debugging
    pub fn raw_read(&mut self, addr: u16) -> u8 {
        unsafe { longfi_sys::SX1276Read(addr) }
    }

    pub fn set_rx(&mut self) {
        unsafe {
            longfi_sys::SX1276SetRx(0);
        }
    }

    pub fn handle_event(&mut self, event: RfEvent) -> ClientEvent {
        unsafe { longfi_sys::longfi_handle_event(&mut self.c_handle, event) }
    }

    pub fn send(&mut self, buffer: &[u8]) {
        unsafe {
            longfi_sys::longfi_send(
                &mut self.c_handle,
                QualityOfService::LONGFI_QOS_0,
                buffer.as_ptr(),
                buffer.len(),
            );
        }
    }

    pub fn send_test(&mut self) {
        unsafe {
            longfi_sys::longfi_rf_test(&mut self.c_handle);
        }
    }

    pub fn get_rx(&mut self) -> RxPacket {
        unsafe { longfi_sys::longfi_get_rx() }
    }

    pub fn get_random(&mut self) -> u32 {
        unsafe { longfi_sys::SX1276Random() }
    }
}

extern crate libm;

#[no_mangle]
pub extern "C" fn ceil(expr: f64) -> f64 {
    libm::ceil(expr)
}

#[no_mangle]
pub extern "C" fn round(expr: f64) -> f64 {
    libm::round(expr)
}

#[no_mangle]
pub extern "C" fn floor(expr: f64) -> f64 {
    libm::floor(expr)
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::{LongFi, RfConfig};
    use longfi_sys::{
        GpioIrqHandler, Gpio_t, IrqModes, IrqPriorities, PinConfigs, PinModes, PinNames, PinTypes,
        Spi_t,
    };

    static mut FIRED: bool = false;
    #[no_mangle]
    pub extern "C" fn spi_in_out(s: *mut Spi_t, out_data: u16) -> u16 {
        unsafe { FIRED = true };
        0
    }
    #[no_mangle]
    pub extern "C" fn gpio_init(
        obj: *mut Gpio_t,
        pin: PinNames,
        mode: PinModes,
        config: PinConfigs,
        pin_type: PinTypes,
        value: u32,
    ) {
    }
    #[no_mangle]
    pub extern "C" fn gpio_write(obj: *mut Gpio_t, value: u32) {}
    #[no_mangle]
    pub extern "C" fn gpio_set_interrupt(
        obj: *mut Gpio_t,
        irq_mode: IrqModes,
        irq_priority: IrqPriorities,
        irq_handler: GpioIrqHandler,
    ) {
    }
    #[no_mangle]
    pub extern "C" fn delay_ms(ms: u32) {}

    static mut BINDINGS: BoardBindings = BoardBindings {
        spi_in_out: Some(spi_in_out),
        delay_ms: Some(delay_ms),
        gpio_init: Some(gpio_init),
        gpio_write: Some(gpio_write),
        gpio_set_interrupt: Some(gpio_set_interrupt),
    };

    #[test]
    fn test_linking() {
        let config = RfConfig {
            oui: 0x12345678,
            device_id: 0x9abc,
        };
        let mut longfi_radio = unsafe { LongFi::new(&mut BINDINGS, config).unwrap() };
        longfi_radio.initialize();

        unsafe { assert!(FIRED) };
    }
}
