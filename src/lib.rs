#![cfg_attr(not(test), no_std)]
use longfi_sys;

pub use longfi_sys::AntPinsMode_t as AntPinsMode;
pub use longfi_sys::BoardBindings_t as BoardBindings;
pub use longfi_sys::ClientEvent_t as ClientEvent;
pub use longfi_sys::LongFiAuthCallbacks as AuthCb;
pub use longfi_sys::LongFiAuthMode_t as AuthMode;
pub use longfi_sys::LongFiConfig_t as Config;
pub use longfi_sys::LongFi_t;
use longfi_sys::Radio_t;
pub use longfi_sys::RfEvent_t as RfEvent;
pub use longfi_sys::RxPacket_t as RxPacket;
pub use longfi_sys::SX126xRadioNew;
pub use longfi_sys::SX1276RadioNew;

static mut SX12XX: Option<Radio> = None;

pub struct LongFi {
    c_handle: LongFi_t,
}

#[derive(Debug)]
pub enum Error {
    NoRadioPointer,
}

unsafe impl Send for LongFi {}

pub struct Radio {
    c_handle: Radio_t
}

impl Radio {
    pub fn sx1262() -> Radio {
        Radio {
            c_handle: unsafe{ SX126xRadioNew() }
        }
    }

    pub fn sx1276() -> Radio {
        Radio {
            c_handle: unsafe { SX1276RadioNew() }
        }
    }
}


impl LongFi {
    pub fn new(
        radio: Radio,
        bindings: &mut BoardBindings,
        config: Config,
        auth_cb_set: &[u8; 16],
    ) -> Result<LongFi, Error> {
        unsafe {
            SX12XX = Some(radio);

            let mut auth_cb = core::mem::zeroed::<AuthCb>();
            *auth_cb.preshared_key.as_mut() = auth_cb_set.as_ptr();

            if let Some(radio) = &mut SX12XX {
                let radio_ptr: *mut Radio_t = &mut radio.c_handle;

                let mut longfi_radio = LongFi {
                    c_handle: longfi_sys::longfi_new_handle(bindings, radio_ptr, config, auth_cb),
                };

                longfi_sys::longfi_init(&mut longfi_radio.c_handle);

                Ok(longfi_radio)
            } else {
                Err(Error::NoRadioPointer)
            }
        }
    }

    pub fn set_buffer(&mut self, buffer: &mut [u8]) {
        unsafe {
            longfi_sys::longfi_set_buf(&mut self.c_handle, buffer.as_mut_ptr(), buffer.len());
        }
    }

    pub fn handle_event(&mut self, event: RfEvent) -> ClientEvent {
        unsafe { longfi_sys::longfi_handle_event(&mut self.c_handle, event) }
    }

    pub fn send(&mut self, buffer: &[u8]) {
        unsafe {
            longfi_sys::longfi_send(&mut self.c_handle, buffer.as_ptr(), buffer.len());
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

    #[no_mangle]
    pub extern "C" fn spi_in_out(s: *mut Spi_t, out_data: u16) -> u16 {
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

    static mut RFCONFIG: RfConfig = RfConfig {
        oui: 0x12345678,
        device_id: 0x9abc,
    };
    static mut BINDINGS: BoardBindings = BoardBindings {
        spi_in_out: Some(spi_in_out),
        delay_ms: Some(delay_ms),
        gpio_init: Some(gpio_init),
        gpio_write: Some(gpio_write),
        gpio_set_interrupt: Some(gpio_set_interrupt),
    };
    static mut LONGFI: Option<LongFi> = None;

    #[test]
    fn test_linking() {
        let mut longfi_radio = unsafe { LongFi::new(&mut BINDINGS, &mut RFCONFIG).unwrap() };
        longfi_radio.initialize();

        unsafe { LONGFI = Some(longfi_radio) };
    }

    #[test]
    fn test_sending() {
        let config = RfConfig {
            oui: 0x12345678,
            device_id: 0x9abc,
        };

        let bindings = BoardBindings {
            spi_in_out: Some(spi_in_out),
            delay_ms: Some(delay_ms),
            gpio_init: Some(gpio_init),
            gpio_write: Some(gpio_write),
            gpio_set_interrupt: Some(gpio_set_interrupt),
        };

        let packet: [u8; 5] = [0xDE, 0xAD, 0xBE, 0xEF, 0];
        unsafe {
            if let Some(longfi) = &mut LONGFI {
                longfi.send(&packet);
            }
        };
    }
}
