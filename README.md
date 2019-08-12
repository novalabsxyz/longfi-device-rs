# longfi-device-rs

## Code Status

[![Build Status](https://travis-ci.com/helium/longfi-device.svg?token=35YrBmyVB8LNrXzjrRop&branch=master)](https://travis-ci.com/helium/longfi-device-rs)


## Summary

This project creates Rust bindings for the [LongFi Protocol C-library](https://github.com/helium/longfi-device).

## Cloning, building, testing

This project contains the LongFi Protocol C-library as a submodule, so either clone the whole repo and submodules with one command:
	`git clone --recurse-submodules git@github.com:helium/longfi-device-rs.git`

Or, if you've already cloned the top level without submodules, get the submodule:
	`git submodule update --init --recursive`

You can use cargo to build:
	`cargo build [--release]`

The configuration in `.cargo/config` is configured to build only for the `thumbv6m-none-eabi` platform currently.

The code in the example directory is for the [STM32L0 Discovery kit](https://www.st.com/en/evaluation-tools/b-l072z-lrwan1.html), which features the [STM32L072CZ](https://www.st.com/en/microcontrollers-microprocessors/stm32l072cz.html).

To upload the code, start a debug server using either JLink (Note: [you can reprogram the ST-Link](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/) on the discovery kit to act like a JLink Server; you will lose the virtual UART over USB provided by the ST-Link):
	`JLinkGDBServer -device STM32L072CZ -speed 4000 -if jtag -AutoConnect -1 -port 3333`

or OpenOCD server (Note: if you are using the OpenOCD server, you will want to update `.cargo/config:runner` to use `openocd.gdb` instead of `jlink.gdb`):
	`openocd -f ./openocd.cfg`

run the example:
	`cargo run --example stm32l0x2 [--release]`

