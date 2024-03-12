#![no_std]
#![no_main]


use cortex_m::asm;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
// use panic_halt as _;
use panic_semihosting as _; // features = ["exit"]
use stm32f7xx_hal::{self as hal, pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let gpiob = dp.GPIOB.split();
    let mut led_red = gpiob.pb14.into_push_pull_output();
    let mut led_blue = gpiob.pb7.into_push_pull_output();

    let mut i = 0;

    loop {

        // assert_ne!(i, 10,"aiuto");
        // panic!("Ops!");
        if i < 5 {
            led_red.toggle();
        }
        else {
            led_blue.toggle();
        }
        i += 1;
		if i == 10 {
			i = 0;
		}
        // led.toggle().unwrap();
        #[cfg(debug_assertions)]
        let _err = hprintln!("Hello World!");    
        asm::delay(2_000_000);
      }
}

