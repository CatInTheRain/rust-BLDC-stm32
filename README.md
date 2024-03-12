## Generate a new project
reference: https://docs.rust-embedded.org/book/start/qemu.html

Actual platform: [link-hardware](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/)

Generate a new project
```bash
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart

Project Name: app
Creating project called `app`...
Done! New project created /tmp/app
```

## Docker
#TODO

---

Let's inspect the config files

### .cargo/config.toml
The most important setting is how to link our executable 
```toml
[target.thumbv7em-none-eabihf]
rustflags = [
  "-C", "link-arg=-Tlink.x",
]
```
and the compilation target architecture
```toml
[build]
target = "thumbv7m-none-eabihf"    # Cortex-M4F
```

You can also set different runners:
```toml
[target.thumbv7em-none-eabi]
runner = "probe-run --chip STM32F767ZITx"
```

### .vscode/*
reference: https://github.com/rust-embedded/cortex-m-quickstart/tree/master/.vscode

Just adapt the example to yours for debug with vscode

### Cargo.toml
Reference: https://github.com/stm32-rs/stm32f3xx-hal/blob/master/README.md
Add stm32XXxx-hal and other dependencies:
```toml
[dependencies]
cortex-m = { version = "0.7.4", features = ["critical-section-single-core"]}
cortex-m-rt = { version = "0.7.3", features = ["device"] }
panic-halt = "0.2.0"
stm32f7xx-hal = { version = "0.7.0", features = ["rt", "stm32f767"] }
```

<!-- ### memory.x
```c
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* TODO Adjust these memory regions to match your device memory layout */
  /* These values correspond to the LM3S6965, one of the few devices QEMU can emulate */
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM : ORIGIN = 0x20000000, LENGTH = 16K
}
These are the start addresses and the memory size of your specific chip
``` -->

---

## Cross-Compilation
Since the thumbv7m-none-eabihf compilation target has been set as the default in your `.cargo/config.toml` file, the two commands below do the same:
```bash
cargo build --target thumbv7m-none-eabihf
cargo build
```
You can inspect the object created:
```bash
cargo readobj --bin hello-rust-embedded -- --file-headers
```
You can also inspect the optimized version
```bash
cargo size --bin hello-rust-embedded --release -- -A
```
Or disassemble the binary
```bash
cargo objdump --bin hello-rust-embedded --release -- --disassemble --no-show-raw-insn --print-imm-hex
```

> IMPORTANT: ELF files contain metadata like debug information so their size on disk does not accurately reflect the space the program will occupy when flashed on a device. Always use cargo-size to check how big a binary really is.

## Running on QEMU
You can run scripts from example folder
```bash
cargo build --example hello_debug
```
The script is the following:
```rust
#![no_main]
#![no_std]

use panic_halt as _;
use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_halt as _;
#[allow(unused_imports)]
use stm32f7xx_hal;
#[entry]
fn main() -> ! {
    hprintln!("Hello, world!").unwrap();

    // exit QEMU
    // NOTE do not run this on hardware; it can corrupt OpenOCD state
    debug::exit(debug::EXIT_SUCCESS);

    loop {
        hprintln!("Hi! I am inside the loop!").unwrap();
        asm::delay(2_000_000);
    }
}

```
It does not work anymore... fix it!

---

## Flash
reference: https://probe.rs/docs/tools/cargo-embed/
### Installing cargo flash is simple:
```bash
cargo install probe-rs --features cli
```
### In your cargo project directory, call
```bash
cargo flash --release --chip <chip_name>
```
### Don't know if your target is supported by cargo flash and what it's name is?
```bash
probe-rs chip list
```
### You can run your examples as usual with
```bash
cargo flash --example <your_example>
```
### Specifying manually what options should be used
```bash
cargo flash --release --chip nRF52840_xxAA --target thumbv6m-none-eabi --example gpio_hal_blinky
```
#### Using a custom chip definition from a non-builtin file
```bash
cargo flash --release --chip-description-path nRF52840_xxAA.yaml --target thumbv6m-none-eabi --example gpio_hal_blinky
```

Example:
```bash
cargo flash --example hello_debug --release --chip STM32F767ZITx
```

## Cargo Embed
reference: https://probe.rs/docs/tools/cargo-embed/
