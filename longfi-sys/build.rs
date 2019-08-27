use bindgen;
use cc;
use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    // build `libloragw`
    Command::new("make")
        .args(&["-C ", "longfi-device/radio/sx1276"])
        .status()
        .expect("sx1276 build failed");

    let radio_path =
        PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("longfi-device/radio/");

    let conf_path =
        PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap()).join("longfi-device/conf");

    println!(
        "cargo:rustc-link-search=native={}",
        radio_path.to_str().unwrap()
    );
    println!(
        "cargo:rustc-link-search=native={}",
        conf_path.to_str().unwrap()
    );

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
       .whitelist_function("board_set_bindings")
       .whitelist_function("SX1276Init")
       .whitelist_function("SX1276GetStatus")
       .whitelist_function("SX1276SetModem")
       .whitelist_function("SX1276SetChannel")
       .whitelist_function("SX1276IsChannelFree")
       .whitelist_function("SX1276Random")
       .whitelist_function("SX1276SetRxConfig")
       .whitelist_function("SX1276SetTxConfig")
       .whitelist_function("SX1276GetTimeOnAir")
       .whitelist_function("SX1276Send")
       .whitelist_function("SX1276SetSleep")
       .whitelist_function("SX1276SetStby")
       .whitelist_function("SX1276SetRx")
       .whitelist_function("SX1276StartCad")
       .whitelist_function("SX1276ReadRssi")
       .whitelist_function("SX1276Write")
       .whitelist_function("SX1276Read")
       .whitelist_function("SX1276WriteBuffer")
       .whitelist_function("SX1276ReadBuffer")
       .whitelist_function("SX1276SetMaxPayloadLength")
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
        .include("longfi-device")
        .include("longfi-device/radio")
        .file("longfi-device/longfi.c")
        .file("longfi-device/board.c")
        // you can change this file to build for a different chip
        // put this in features later
        .file("longfi-device/radio/sx1276/sx1276-board.c")
        .file("longfi-device/radio/sx1276/sx1276.c")
        .compile("longfi-device");
}
