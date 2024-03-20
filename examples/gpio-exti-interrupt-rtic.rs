#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
// use panic_halt as _;

// peripherals = true makes sure that the device handle/field is available for use later in our code
#[rtic::app(device = stm32f7xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    #[cfg(debug_assertions)]
    use cortex_m_semihosting::hprintln;
    use stm32f7xx_hal::{
        gpio::{self, Edge, Input, Output, PushPull, ExtiPin},
        pac::TIM2,
        prelude::*,
        timer::{self, Event},
    };

    // Resources shared between tasks
    #[shared]
    struct Shared {
        timer: timer::CounterUs<TIM2>,
    }

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        button: gpio::PC13<Input>,
        led_blue: gpio::PB7<Output<PushPull>>,
        led_red: gpio::PB14<Output<PushPull>>,
        delayval: u32,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = ctx.device;

        // Promote RCC structure to HAL to be able to configure clocks
        let mut rcc = dp.RCC.constrain();
        // Configure the system clocks
        let clocks = rcc.cfgr.sysclk(216.MHz()).freeze(); // freeze the clock configuration
    
        // Configure the LED_blue pin as a push pull ouput and obtain handle
        // On the Nucleo F767 theres an on-board LED_blue connected to pin PB7
        // 1) Promote the GPIOA PAC struct
        let gpiob = dp.GPIOB.split();
        // 2) Configure Pin and Obtain Handle
        let led_blue = gpiob.pb7.into_push_pull_output();
        let led_red = gpiob.pb14.into_push_pull_output();
    
        // Configure the button pin as input and obtain handle
        // On the Nucleo F767 there is a button connected to pin PC13
        // 1) Promote the GPIOC PAC struct
        let gpioc = dp.GPIOC.split();
        // 2) Configure Pin and Obtain Handle
        let mut button = gpioc.pc13;
    
        // Configure Button Pin for Interrupts
        // 1) Make button an interrupt source
        button.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
        // 2) Trigger only on the rising edge
        let mut exti = dp.EXTI;
        button.trigger_on_edge(&mut exti, Edge::Rising);
        // 3) Enable GPIO interrupt for button
        button.enable_interrupt(&mut exti);
    
        // Create delay handle
        //let mut delay = dp.TIM1.delay_ms(&clocks);
        let mut timer = dp.TIM2.counter_us(&clocks);

        // Kick off the timer with 2 seconds timeout first
        // It probably would be better to use the global variable here but I did not to avoid the clutter of having to create a crtical section
        timer.start(2000.millis()).unwrap();

        // Set up to generate interrupt when timer expires
        timer.listen(Event::Update);

        // Call a software task that blink led_red
        sw_task::spawn().unwrap();

        (
            // Initialization of shared resources
            Shared { timer },
            // Initialization of task local resources
            Local {
                button,
                led_blue,
                led_red,
                delayval: 2000_u32,
            },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling (remove it after rtic v2.0.0)
            init::Monotonics(),
        )
    }

    // You can create also software task
    #[task(priority = 1, local=[led_red])]
    fn sw_task(ctx: sw_task::Context) {
        ctx.local.led_red.set_high();
        #[cfg(debug_assertions)]
        hprintln!("Hello From sw_task").unwrap();
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Go to sleep
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = EXTI15_10, local = [delayval, button], shared=[timer])]
    fn button_pressed(mut ctx: button_pressed::Context) {
        // When Button press interrupt happens three things need to be done
        // 1) Adjust Global Delay Variable
        // 2) Update Timer with new Global Delay value
        // 3) Clear Button Pending Interrupt

        // Obtain a copy of the delay value from the global context
        let mut delay = *ctx.local.delayval;

        // Adjust the amount of delay
        delay = delay - 500_u32;
        if delay < 500_u32 {
            delay = 2000_u32;
        }

        // Update delay value in global context
        *ctx.local.delayval = delay;

        // Update the timeout value in the timer peripheral
        ctx.shared
            .timer
            .lock(|tim| tim.start(delay.millis()).unwrap());

        // Obtain access to Button Peripheral and Clear Interrupt Pending Flag
        ctx.local.button.clear_interrupt_pending_bit();
    }

    #[task(binds = TIM2, local=[led_blue], shared=[timer])]
    fn timer_expired(mut ctx: timer_expired::Context) {
        // When Timer Interrupt Happens Two Things Need to be Done
        // 1) Toggle the LED_blue
        // 2) Clear Timer Pending Interrupt

        ctx.local.led_blue.toggle();
        ctx.shared
            .timer
            .lock(|tim| tim.clear_interrupt(Event::Update));
    }
}