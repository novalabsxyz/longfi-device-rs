#![no_std]
use longfi_sys;

pub use longfi_sys::ClientEvent;
pub use longfi_sys::QualityOfService;
pub use longfi_sys::RfConfig;
pub use longfi_sys::RfEvent;
pub use longfi_sys::RxPacket;

#[macro_use(singleton)]
extern crate cortex_m;

pub struct LongFi;

impl LongFi {

    pub fn new() -> LongFi{
        // OK if LongFi is only instantiable once
        let x: &'static mut bool =
            singleton!(: bool = false).unwrap();

        LongFi {}
    }

    pub fn enable_tcxo(&mut self) {
        unsafe {
            longfi_sys::longfi_enable_tcxo();
        }
    }

    pub fn initialize(&mut self, config: RfConfig) {
        unsafe {
            longfi_sys::longfi_init(config);
        }
    }

    pub fn set_buffer(&mut self, buffer: &mut [u8]) {
        unsafe {
            longfi_sys::longfi_set_buf(buffer.as_mut_ptr(), buffer.len());
        }
    }

    //for debugging
    pub fn raw_read(&mut self, addr: u8) -> u8 {
        unsafe { longfi_sys::SX1276Read(addr) }
    }

    pub fn set_rx(&mut self) {
        unsafe {
            longfi_sys::SX1276SetRx(0);
        }
    }

    pub fn handle_event(&mut self, event: RfEvent) -> ClientEvent {
        unsafe { longfi_sys::longfi_handle_event(event) }
    }

    pub fn send(&mut self, buffer: &[u8], len: usize) {
        let send_len = ::core::cmp::min(len, buffer.len());
        unsafe {
            longfi_sys::longfi_send(buffer.as_ptr(), send_len);
        }
    }

    pub fn send_test(&mut self) {
        unsafe {
            longfi_sys::longfi_rf_test();
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
