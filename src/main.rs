#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
// use panic_halt as _;

// peripherals = true makes sure that the device handle/field is available for use later in our code
#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use core::{borrow::BorrowMut, fmt::Error};
    use cortex_m::delay::Delay;
    // #[cfg(debug_assertions)]
    use cortex_m_semihosting::hprintln;
    use hal::adc;
    use nb;
    use stm32f3xx_hal::{
		self as hal,
		gpio::{self, Edge, Input, Output, PushPull, Analog},
		pac,
		interrupt,
		prelude::*,
		timer::{self, Timer, Event},
		time::fixed_point::FixedPoint,
        adc::{Adc},
		// dma::{self, DMA}, gpio::{self, Analog, Edge, ExtiPin, Input, Output, PushPull},
		// pac::{ADC1, TIM2, TIM5}, prelude::*, rcc::Clocks,
		// timer::{self, Event}
    };
    // Resources shared between tasks
    #[shared]
    struct Shared {
        // timer: timer::CounterUs<TIM2>,
    }
    
    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        button: gpio::Pin<gpio::Gpioc, gpio::U<13>, Input>,
        led: gpio::PB13<Output<PushPull>>,
        delayval: u32,
        // counter: timer::CounterUs<TIM5>,
        potmeter: gpio::Pin<gpio::Gpioa, gpio::U<0>, Analog>,
        // adc: Adc<pac::ADC1>,
    }
// TODO implement timer handler to measure time
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
		// RTIC setups already the handler for device peripherals
		let dp = ctx.device;
		// Configure and obtain handle for delay abstraction
		// 1) Promote RCC structure to HAL to be able to configure clocks
		let mut rcc = dp.RCC.constrain();
		// 2) Configure the system clocks
		// 216 MHz is the maximum frequency for HSE
		let clocks = rcc
			.cfgr
			.use_hse(8.MHz())
			.sysclk(72.MHz())
			.hclk(72.MHz())
			.pclk1(36.MHz())
			.pclk2(72.MHz())
			.freeze(&mut dp.FLASH.constrain().acr); // freeze the clock configuration
		// you can check if the AHB clock frequency is the desired
		assert!(clocks.hclk().integer() == 72 * u32::pow(10, 6));

        /* LED CONFIGURATION */
        // 1) Promote the GPIOB PAC struct
        let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
        // 2) Configure Pin and Obtain Handle
        let mut led = gpiob.pb13.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        // Turn on led red (led will turn off when the initialization will finish)
        led.set_high().unwrap();
        
        /* BUTTON INTERRUPT CONFIGURATION */
		let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
		let mut exti = dp.EXTI;
        // Configure Button Pin for Interrupts
        let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);
        let mut button = gpioc.pc13.into_pull_down_input(&mut gpioc.moder, &mut gpioc.pupdr);
		// 1) Make button an interrupt source
        syscfg.select_exti_interrupt_source(&button);
		// 2) Trigger only on the rising edge
		button.trigger_on_edge(&mut exti, Edge::Rising);
		// 3) Enable GPIO interrupt for button
		button.enable_interrupt(&mut exti);


        // /* ADC CONFIGURATION */
		// Create a Common ADC instance
		let adc_common = adc::CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);
		// Set up ADC1
		let mut adc = adc::Adc::new(dp.ADC1, adc::config::Config::default(), &clocks, &adc_common)
		// Convert the ADC into `OneShot` mode.
		.into_oneshot();
		// Set up pin PA0 as analog pin
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
		let mut potmeter = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
		let adc_data: u16 = adc.read(&mut potmeter).unwrap();
        // let potmeter = gpioa.pa3.into_analog();
        // // Configure ADC for sequence conversion with interrupts
        // let mut adc = Adc::adc1(dp.ADC1, &mut rcc.apb2, &clocks, 12, true);
        // // set the sample time to set the total conversion time
        // adc.set_sample_time(SampleTime::T_28); // with the max frequency the ADCCLK is 108MHz / 4 ?
        // // adc.convert();
        // // hprintln!("{}", clocks.pclk2().raw());
        // // adc.set_external_trigger();
        // //expected: 1093 micros
        
        // /* TIMER CONFIGURATION */
        // let counter = dp.TIM5.counter_us(&clocks);

        // // Create delay handle
        // //let mut delay = dp.TIM1.delay_ms(&clocks);
        // let mut timer = dp.TIM2.counter_us(&clocks);

        // // Kick off the timer with 2 seconds timeout first
        // // It probably would be better to use the global variable here but I did not to avoid the clutter of having to create a crtical section
        // timer.start(2000.millis()).unwrap();

        // // Set up to generate interrupt when timer expires
        // timer.listen(Event::Update);

        led.set_low().unwrap();
        // Call a software task that blink led_red
        // sw_task::spawn().unwrap();

        (
            // Initialization of shared resources
            Shared { 
                // timer,
            },
            // Initialization of task local resources
            Local {
                button,
				led,
                delayval: 2000_u32,
                // counter,
                potmeter,
                // adc,
            },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling (remove it after rtic v2.0.0)
            // init::Monotonics(),
        )
    }

    // You can create also software task
    // #[task(priority = 1, local=[led])]
    // async fn sw_task(ctx: sw_task::Context) {
    //     ctx.local.led_red.set_low();
    //     #[cfg(debug_assertions)]
    //     hprintln!("Hello From sw_task").unwrap();
    // }

    // Background task, runs whenever no other tasks are running
    #[idle(local = [led, potmeter])]
    fn idle(ctx: idle::Context) -> ! {
        loop {
			cortex_m::asm::delay(8_000_000);
			ctx.local.led.set_high().unwrap();
			cortex_m::asm::delay(8_000_000);
			ctx.local.led.set_low().unwrap();
			
            // Go to sleep
            // cortex_m::asm::wfi();
        }
    }

    #[task(binds = EXTI15_10, local = [button])]
    fn button_pressed(ctx: button_pressed::Context) {
		hprintln!("Button Pressed!").unwrap();
		ctx.local.button.clear_interrupt();
        // When Button press interrupt happens three things need to be done
        // 1) Adjust Global Delay Variable
        // 2) Update Timer with new Global Delay value
        // 3) Clear Button Pending Interrupt

        // Obtain a copy of the delay value from the global context
        // let mut delay = *ctx.local.delayval;
    }

    // #[task(binds = TIM2)]
    // fn timer_expired(mut ctx: timer_expired::Context) {
        // // When Timer Interrupt Happens Two Things Need to be Done
        // // 1) Toggle the LED_blue
        // // 2) Clear Timer Pending Interrupt
        // ctx.local.counter.start(100.secs()).unwrap();

        // ctx.local.led.toggle();
        // ctx.shared
        //     .timer
        //     .lock(|tim| tim.clear_interrupt(Event::Update));
        // // let elapsed_time = ctx.local.counter.now().duration_since_epoch().to_micros();
        // // ctx.local.counter.cancel().unwrap();
        // #[cfg(debug_assertions)]
        // hprintln!("Time elapsed: {}", elapsed_time).unwrap();


        // convert(&temperature_pin, SampleTime::T_28);

    // }
}

// // IMports
// use core::cell::{Cell, RefCell};
// use cortex_m::interrupt::Mutex;
// use cortex_m_rt::entry;
// use cortex_m::peripheral::NVIC;
// // use panic_semihosting as _;
// use panic_halt as _; //more optimized!
// use cortex_m_semihosting::hprintln;
// use stm32f3xx_hal::{
//     gpio::{self, Edge, Input, Output, PushPull},
//     pac::{self, interrupt},
//     prelude::*, time::fixed_point::FixedPoint,
// };
