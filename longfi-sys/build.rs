#[cfg(workaround_build)]
fn main() {
    use std::env;
    use std::path::PathBuf;
    use std::process::Command;

   // make the bindings
   let bindings = bindgen::Builder::default()
       .raw_line("use cty;")
       .use_core()
       .ctypes_prefix("cty")
       .header("longfi-device/board.h")
       .header("longfi-device/longfi.h")
       .header("longfi-device/radio/radio.h")
       .header("longfi-device/radio/sx1276/sx1276.h")
       .whitelist_var("XTAL_FREQ")
       .whitelist_var("FREQ_STEP")
       .whitelist_var("RX_BUFFER_SIZE")
       .whitelist_type("RfEvent")
       .whitelist_type("RadioState")
       .whitelist_type("RadioModems")
       .whitelist_type("ClientEvent")
       .whitelist_type("QualityOfService")
       .whitelist_type("RfConfig")
       .whitelist_type("RxPacket")
       .whitelist_function("longfi_init")
       .whitelist_function("longfi_new_handle")
       .whitelist_function("longfi_handle_event")
       .whitelist_function("longfi_send")
       .whitelist_function("longfi_get_rx")
       .whitelist_function("longfi_set_buf")
       .whitelist_function("longfi_rf_test")
       .whitelist_function("longfi_get_random")
       .whitelist_function("longfi_enable_tcxo")
       .whitelist_function("board_set_bindings")
       .whitelist_function("SX1276RadioNew")
       .trust_clang_mangling(false)
       .rustfmt_bindings(true)
       .rustified_enum("ClientEvent")
       .rustified_enum("RfEvent")
       .rustified_enum("QualityOfService")
       .derive_copy(false)
       .derive_debug(false)
       .layout_tests(false)
       .generate()
       .expect("Failed to generate sx1276 bindings!");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    cc::Build::new()
        .pic(false)
        .flag("-std=gnu99")
        .include("longfi-device/")
        .include("longfi-device/radio")
        .file("longfi-device/longfi.c")
        .file("longfi-device/board.c")
        // you can change this file to build for a different chip
        // put this in features later
        .file("longfi-device/radio/sx1276/sx1276-board.c")
        .file("longfi-device/radio/sx1276/sx1276.c")
        .compile("longfi-device");
}

#[cfg(not(workaround_build))]
fn main() {
    cargo_5730::run_build_script();
}
