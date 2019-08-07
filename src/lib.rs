#![no_std]
use longfi_sys;

pub use longfi_sys::ClientEvent;
pub use longfi_sys::QualityOfService;
pub use longfi_sys::RfConfig;
pub use longfi_sys::RfEvent;
pub use longfi_sys::RxPacket;

pub struct LongFi;

impl LongFi {
    pub fn enable_tcxo() {
        unsafe {
            longfi_sys::longfi_enable_tcxo();
        }
    }

    pub fn initialize(config: RfConfig) {
        unsafe {
            longfi_sys::longfi_init(config);
        }
    }

    pub fn set_buffer(buffer: &mut [u8]) {
        unsafe {
            longfi_sys::longfi_set_buf(buffer.as_mut_ptr(), buffer.len());
        }
    }

    //for debugging
    pub fn raw_read(addr: u8) -> u8 {
        unsafe { longfi_sys::SX1276Read(addr) }
    }

    pub fn set_rx() {
        unsafe {
            longfi_sys::SX1276SetRx(0);
        }
    }

    pub fn handle_event(event: RfEvent) -> ClientEvent {
        unsafe { longfi_sys::longfi_handle_event(event) }
    }

    pub fn send(buffer: &[u8], len: usize) {
        let send_len = ::core::cmp::min(len, buffer.len());
        unsafe {
            longfi_sys::longfi_send(buffer.as_ptr(), send_len);
        }
    }

    pub fn send_test() {
        unsafe {
            longfi_sys::longfi_rf_test();
        }
    }

    pub fn get_rx() -> RxPacket {
        unsafe { longfi_sys::longfi_get_rx() }
    }

    pub fn get_random() -> u32 {
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
