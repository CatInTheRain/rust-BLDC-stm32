# Get started on Linux
## Requirements
- Rust (Rust Compiler was version 1.76.0 at time of development)
- STM32 micro-controller (in this project I used [this stm32f7](https://os.mbed.com/platforms/ST-Nucleo-F767ZI/))
- One potentiometer to control the duty cycle 
- Esc with pre-installed firmware (generally a variation of BLHeli)
- An appropriate brushless DC electric motor

## Classic Installation
```bash
# Install Linux Requirements
sudo apt update
sudo apt install build-essential cmake pkg-config libssl-dev libudev-dev curl
# Install and source Rust with Cargo
curl https://sh.rustup.rs -sSf | bash -s -- -y
source $HOME/.cargo/env
# Install Rust dependencies for generic stm32 development
rustup update
rustup component add llvm-tools-preview
cargo install cargo-binutils cargo-expand cargo-generate
cargo install probe-rs --features cli
# Depending of the machine architecture intall the appropriate compilation target
rustup target add thumbv7em-none-eabihf # for Cortex-M7F (with FPU)
# Linux General Dependencies for stm32 development
sudo apt-get install gdb-multiarch openocd qemu-system-arm
```
Finally clone this repository

```bash
git clone https://github.com/CatInTheRain/rust-BLDC-stm32.git
```

## Docker Installation (Alternative)
Clone this repository
```bash
git clone https://github.com/CatInTheRain/rust-BLDC-stm32.git
```
build the docker image
```bash
./docker/build.sh
```
and finally run the container
```bash
./docker/go.sh
```
The repository will be mounted inside the docker container, (every modification in the docker image will be also in the local package).

## Build the project

Now you can compile the package and all of its dependencies
```bash
git clone https://github.com/CatInTheRain/rust-BLDC-stm32.git
cd rust-BLDC-stm32
cargo build
# or if you want to build the release version
cargo build --release
```

# Pinout of this Project
| Header stm32 | <---> color |      Header HW     | Function |
|:------------:|:-----------:|:------------------:|:--------:|
|     3.3V     |     red     |   IN Potentometer  |          |
|      GND     |    white    |  GND Potentometer  |          |
|      PA3     |    orange   | Wiper Potentometer |    ADC   |
|      GND     | black/green |       GND ESC      |          |
|      PE5     | blue/yellow |     Signal ESC     |    PWM   |

For this project, I used a 4S LiPo battery which has 4 cells and that’s 16.8V when it is charged. 

![](media/pinout.jpg)


# Flash this project

If you have the same board of this project you can simply connect the STM32 to the PC and run

```bash
cargo flash --release --chip STM32F767ZITx
```
> If you have another board you can follow the readme [here](#how-to-setup-the-project-according-to-your-stm32) to set the config files.

---

# Project Explanation
The task of this project is to control the speed of the brushless motor of zero to maximum using the potentiometer. In addition if the user clicks the button the pwm generator will be disabled by an interrupt, to disable the motor, until che microcontroler is reset by the reset button.

The following video demonstrates this task.

https://github.com/CatInTheRain/rust-BLDC-stm32/assets/55113554/2415d398-1e7d-4431-9319-9d9dbee1cc24


TODO: code discussion and difference in using or not RTIC
TODO: other elements of interest

---

# How to setup the project according to a different stm32?

> Rust libraries of different stm32 can change the syntax, study the respective crate that you are using.

Let's inspect the config files of the repository

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
[target.thumbv7em-none-eabihf]
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

### memory.x 
For some boards it is mandatory to create a `memory.x` file with the informations about the start addresses and the memory size of your specific chip
```c
MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* TODO Adjust these memory regions to match your device memory layout */
  /* These values correspond to the LM3S6965, one of the few devices QEMU can emulate */
  FLASH : ORIGIN = 0x08000000, LENGTH = 64K
  RAM : ORIGIN = 0x20000000, LENGTH = 16K
}
```

---

## Cross-Compilation
Since the `
thumbv7m-none-eabihf` compilation target has been set as the default in your `.cargo/config.toml` file, the two commands below do the same:
```bash
cargo build --target thumbv7m-none-eabihf
cargo build
```
You can inspect the object created:
```bash
cargo readobj --bin rust_bldc_stm32 -- --file-headers
```
You can also inspect the optimized version
```bash
cargo size --bin rust_bldc_stm32 --release -- -A
```
Or disassemble the binary
```bash
cargo objdump --bin rust_bldc_stm32 --release -- --disassemble --no-show-raw-insn --print-imm-hex
```

> IMPORTANT: ELF files contain metadata like debug information so their size on disk does not accurately reflect the space the program will occupy when flashed on a device. Always use cargo-size to check how big a binary really is.

## Running on QEMU
You can run scripts for generic devices, see [this book](https://docs.rust-embedded.org/book/) for more details

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

# Troubleshoot
Sometimes there could be some error when you want to debug the code; usually this happens after user flash code with:

```bash
cargo flash --example WIP --release --chip STM32F767ZITx
```

```console
[...]
Please check TERMINAL tab (gdb-server) for output from openocd
Finished reading symbols from objdump: Time: 17 ms
Finished reading symbols from nm: Time: 14 ms
Output radix now set to decimal 10, hex a, octal 12.
Input radix now set to decimal 10, hex a, octal 12.
Failed to launch GDB: Remote connection closed (from target-select extended-remote localhost:50000)
```

To fix this problem you have to reflash the code in debug mode:
```bash
cargo flash --example WIP --chip STM32F767ZITx --connect-under-reset
```
Then it is possible to debugb the code again
