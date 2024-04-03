#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_semihosting as _;
// use panic_halt as _; // you can uncomment if you don't want to debug

pub struct EscConf {
    min_throttle: u32,
    max_throttle: u32,
}

// peripherals = true makes sure that the device handle/field is available for use later in our code
#[rtic::app(device = stm32f7xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    #[cfg(debug_assertions)]
    use cortex_m_semihosting::hprintln;
    use core::borrow::BorrowMut;
    use stm32f7xx_hal::{
        adc::{Adc, SampleTime},
        gpio::{self, Analog, Edge, ExtiPin, Input, Output, PushPull},
        pac::{ADC1, TIM2, TIM5, TIM9},
        prelude::*,
        timer::{self, Event, PwmChannel}
    };

    use crate::EscConf;

    // Resources shared between tasks
    #[shared]
    struct Shared {
        timer: timer::CounterUs<TIM2>,
        pwm_ch1: PwmChannel<TIM9, 0>,
    }
    
    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        button: gpio::PC13<Input>,
        led_blue: gpio::PB7<Output<PushPull>>,
        led_red: gpio::PB14<Output<PushPull>>,
        counter: timer::CounterUs<TIM5>,
        potmeter: gpio::PA3<Analog>,
        adc: Adc<ADC1>,
        esc_conf: EscConf,
    }

// TODO implement timer handler to measure time
    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut dp = ctx.device;

        /* CLOCKS CONFIGURATION */
        let sysclk_freq: u32 = 216;
        // Promote RCC structure to HAL to be able to configure clocks
        let mut rcc = dp.RCC.constrain();
        // Configure the system clocks
        let clocks = rcc.cfgr.sysclk(sysclk_freq.MHz()).freeze(); // freeze the clock configuration
        assert!(clocks.hclk().raw() == sysclk_freq * u32::pow(10, 6));

        /* LED CONFIGURATION */
        // 1) Promote the GPIOB PAC struct
        let gpiob = dp.GPIOB.split();
        // 2) Configure Pin and Obtain Handle
        let led_blue = gpiob.pb7.into_push_pull_output();
        let mut led_red = gpiob.pb14.into_push_pull_output();
        
        // Turn on led red (led will turn off when the initialization will finish)
        led_red.set_high();

        /* BUTTON CONFIGURATION */
        // 1) Promote the GPIOC PAC struct
        let gpioc = dp.GPIOC.split();
        // 2) Configure Pin and Obtain Handle
        let mut button = gpioc.pc13;
        
        /* ADC CONFIGURATION */
        let gpioa = dp.GPIOA.split();
        let potmeter = gpioa.pa3.into_analog();
        // Configure ADC for sequence conversion with interrupts
        let mut adc = Adc::adc1(dp.ADC1, &mut rcc.apb2, &clocks, 12, true);
        // set the sample time to set the total conversion time
        adc.set_sample_time(SampleTime::T_28); // with the max frequency the ADCCLK is 108MHz / 4 ?


        /* BUTTON INTERRUPT CONFIGURATION */
        // Configure Button Pin for Interrupts
        // 1) Make button an interrupt source
        button.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
        // 2) Trigger only on the rising edge
        let mut exti = dp.EXTI;
        button.trigger_on_edge(&mut exti, Edge::Rising);
        // 3) Enable GPIO interrupt for button
        button.enable_interrupt(&mut exti);
        
        /* TIMER CONFIGURATION */
        let counter = dp.TIM5.counter_us(&clocks);

        // Create delay handle
        //let mut delay = dp.TIM1.delay_ms(&clocks);
        // Create timer handle
        let mut timer = dp.TIM2.counter_us(&clocks);

        // Set Timer period to 20ms --> 50 Hz, for esc control
        timer.start(20.millis()).unwrap();

        // Set up to generate interrupt when timer expires
        timer.listen(Event::Update);

        /* PWM CONFIGURATION */
        // I choose the PWM 9/1 (Timer number) / (Channel) --> PE5
        let gpioe = dp.GPIOE.split();
        let esc = gpioe.pe5.into_alternate();
        
        // Now you have to obtain a pwm handle specifying the right type!
        // The below 2 lines explain the complexity and represent a possible solution
        // let mut pwm: PwmHz<TIM9, Ch<0>, gpio::PE5<Alternate<3>>> = dp.TIM9.pwm_hz(esc, 50.Hz(), &clocks); // period of 20ms
        // let mut pwm_ch1: PwmChannel<TIM9, 0> = pwm.split();
        // passing one Pin (in case of multiple pin I can pass a tuple) to the pwm extension trait I obtain
        // the struct PwmHz. The problem is that does not implement the copy trait so I can't use the method
        // let max_duty = pwm.get_max_duty();
        // To solve this you have to call the split method in order to obtain the handle of type PwmChannel
        // for the specific channel (in this case Channel1)
        // If you are not interested in using the copy trait you can call method from the struct PwmHz, but you have
        // to specify the channel: e.g. pwm.set_duty(timer::Channel::C1 ,min_throttle);
        // In case of multiple pins:
        // let channels = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate());
        // let pwm = dp.TIM1.pwm_hz(channels, 20.kHz(), &clocks).split();
        // let (mut ch1, _ch2) = pwm;

        let mut pwm_ch1: PwmChannel<TIM9, 0> = dp.TIM9.pwm_hz(esc, 50.Hz(), &clocks).split();
        // Configure the duty cycle to min throttle
        let max_duty = pwm_ch1.get_max_duty();
        let esc_conf: EscConf = EscConf {
            min_throttle: (max_duty as f32 * 5.0 / 100.0) as u32,  // 1ms => 5% duty cycle
            max_throttle: (max_duty as f32 * 10.0 / 100.0) as u32  // 2ms => 10% duty cycle
        };
        // set duty of the esc to minimum and enable it
        pwm_ch1.set_duty(esc_conf.min_throttle as u16);
        pwm_ch1.enable();

        /* CONFIGURATION END */    
        led_red.set_low();

        (
            // Initialization of shared resources
            Shared { 
                timer,
                pwm_ch1,
            },
            // Initialization of task local resources
            Local {
                button,
                led_blue,
                led_red,
                counter,
                potmeter,
                adc,
                esc_conf
            },
            // Move the monotonic timer to the RTIC run-time,
            // this enables scheduling (remove it after rtic v2.0.0)
            // init::Monotonics(),
        )
    }

    // You can create also software task
    // #[task(priority = 1, local=[led_red])]
    // async fn sw_task(ctx: sw_task::Context) {
    //     ctx.local.led_red.set_low();
    //     #[cfg(debug_assertions)]
    //     hprintln!("Hello From sw_task").unwrap();
    // }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Go to sleep (Wait For Interrupts)
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = EXTI15_10, local = [button, led_red], shared=[timer, pwm_ch1])]
    fn button_pressed(mut ctx: button_pressed::Context) {
        // When Button press interrupt happens three things need to be done
        // 1) Disable PWM channel
        ctx.shared.pwm_ch1.lock(|pwm1| 
            pwm1.disable());
        // 2) Turn on red led
        ctx.local.led_red.set_high();
        // 3) clear interrupt flag to allow consecutive interrupts to happen
        ctx.local.button.clear_interrupt_pending_bit();
    }

    #[task(binds = TIM2, local=[led_blue, counter, potmeter, adc, esc_conf], shared=[timer, pwm_ch1])]
    fn timer_expired(mut ctx: timer_expired::Context) {
        // When Timer Interrupt Happens several Things Need to be Done
        // 1) Clear Timer Pending Interrupt
        ctx.shared.timer.lock(|tim| 
            tim.clear_interrupt(Event::Update));
        // 2) Toggle the LED_blue
        ctx.local.led_blue.toggle();
        
        // optional) If you want you can measure time in micro seconds to measure code execution time 
        ctx.local.counter.start(100.secs()).unwrap();
        
        // 3) Get adc data from the potentometer
        let adc_data: u16 = ctx.local.adc.read(ctx.local.potmeter.borrow_mut()).unwrap();

        // 4) Map the adc value: from A--B to C---D the formula is the following:
        // (X - A) * (D - C) / (B - A) + C, with A = 0, B = 2^12-1, C = min_throttle, D = max_throttle
        // let duty_set = ( (adc_data as u32 - 0) * (6545 - 3272) / (4095 - 0) + 3272 ) as u16;
        let duty_set = ( 
            ( adc_data as u32 * (ctx.local.esc_conf.max_throttle - ctx.local.esc_conf.min_throttle) / 
            4095 ) + ctx.local.esc_conf.min_throttle 
        ) as u16;
        // 5) set pwm duty, accessing to the shared variable
        ctx.shared.pwm_ch1.lock(|pwm1| 
            pwm1.set_duty(duty_set));

        // optional) get the execution time
        let elapsed_time = ctx.local.counter.now().duration_since_epoch().to_micros();
        ctx.local.counter.cancel().unwrap();
        // Time can be printed only in debug mode
        #[cfg(debug_assertions)]
        hprintln!("Time elapsed: {}; adc_data: {}, duty set: {}", elapsed_time, adc_data, duty_set).unwrap();
    }
}