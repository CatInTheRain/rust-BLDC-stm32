// In this case the bouncing problem is fixed without using EXTI
#![no_std]
#![no_main]

// IMports
use core::{cell::{Cell, RefCell}};
use cortex_m::{interrupt::Mutex};
use cortex_m_rt::entry;
use cortex_m::peripheral::NVIC;
use panic_semihosting as _;
// use panic_halt as _; //more optimized!
// use cortex_m_semihosting::hprintln;

use stm32f7xx_hal::{
    gpio::{self, Input},
    pac::{self, interrupt, Interrupt, TIM2, TIM5},
    prelude::*,
    timer::{CounterUs, Event},
};

// Create an alias for pin PC13 and PB14
type ButtonPin = gpio::PC13<Input>;
type LedButton = gpio::Pin<'B', 14, gpio::Output>;

// Global Variable Definitions

static G_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));

// Cell only permits taking a copy of the current value or replacing it
static G_DELAYMS: Mutex<Cell<u32>> = Mutex::new(Cell::new(2000_u32));

static G_LED_RED: Mutex<RefCell<Option<LedButton>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM2: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM5: Mutex<RefCell<Option<CounterUs<TIM5>>>> = Mutex::new(RefCell::new(None));

static PREV_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static CURR_MS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    /* Parameters */ 
    let sysclk_freq: u32 = 216; // MHz

    // Setup handler for device peripherals: take the PAC level device peripherals
    let dp = pac::Peripherals::take().unwrap();
    // Configure and obtain handle for delay abstraction
    // 1) Promote RCC structure to HAL to be able to configure clocks
    let rcc = dp.RCC.constrain();
    // 2) Configure the system clocks
    // 216 MHz is the maximum frequency for HSE
    let clocks = rcc.cfgr.sysclk(sysclk_freq.MHz()).freeze(); // freeze the clock configuration
    // you can check if the AHB clock frequency is the desired
    assert!(clocks.hclk().raw() == sysclk_freq * u32::pow(10, 6));

    // 3) Create delay handle (according to the doc I choose a basic 16-bit timer)
    let mut delay = dp.TIM6.delay_us(&clocks);

    // Configure the LED pin as a push pull ouput and obtain handle
    // On the Nucleo F767 there is an on-board LED connected to pin PB7
    // 1) Promote the GPIOB PAC struct
    let gpiob = dp.GPIOB.split();
    // 2) Configure Pin and Obtain Handle for the pins, so we can control it
    let mut led_blue = gpiob.pb7.into_push_pull_output();

    // Configure another Pin for another led
    let led_red = gpiob.pb14.into_push_pull_output();
    cortex_m::interrupt::free(|cs| {
        G_LED_RED.borrow(cs).replace(Some(led_red));
    });
    // Configure the button pin as input and obtain handle
    // On the Nucleo F767 there is a button connected to pin PC13
    // 1) Promote the GPIOC PAC struct
    let gpioc = dp.GPIOC.split();
    // 2) Configure Pin and Obtain Handle
    let button = gpioc.pc13.into_floating_input();
    // Now that button is configured, move button into global context
    cortex_m::interrupt::free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(button));
    });

    // Set up a timer expiring after 100ms
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start(100.millis()).unwrap();
    // Generate an interrupt when the timer expires
    timer.listen(Event::Update);
    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| {
        G_TIM2.borrow(cs).replace(Some(timer));
    });
    // Enable the interrupt in the NVIC by passing the button interrupt number
    unsafe {
        NVIC::unmask(Interrupt::TIM2);
    }

    // Create a Millisecond Counter Handle
    let counter = dp.TIM5.counter_us(&clocks);  
    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| {
        G_TIM5.borrow(cs).replace(Some(counter));
    });

    cortex_m::interrupt::free(|cs| {
        let mut cnt = G_TIM5.borrow(cs).borrow_mut();
        let _ = cnt.as_mut().unwrap().start(100.secs());
    });

    // Application Loop
    loop{
        // Turn On LED
        led_blue.set_high();
        // Obtain G_DELAYMS and delay
        delay.delay_ms(cortex_m::interrupt::free(|cs| G_DELAYMS.borrow(cs).get()));
        // Turn off LED
        led_blue.set_low();
        // Obtain G_DELAYMS and delay
        delay.delay_ms(cortex_m::interrupt::free(|cs| G_DELAYMS.borrow(cs).get()));
        // Obtain G_DELAYMS and delay
        // cortex_m::interrupt::free(|cs| {
        //     let mut cnt = G_TIM5.borrow(cs).borrow_mut();
        //     let _duration = cnt.as_mut().unwrap().now().duration_since_epoch().to_millis();
        //     cnt.as_mut().unwrap().cancel().unwrap();
        // });
    }
}

#[interrupt]
fn TIM2() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain Access to Delay Global Data and Adjust Delay
        
        // Obtain access to Global Button Peripheral and Clear Interrupt Pending Flag
        // The following line obtains a mutable reference to the Option in G_BUTTON using the borrow_mut method
        let mut button = G_BUTTON.borrow(cs).borrow_mut();
        let mut led_red = G_LED_RED.borrow(cs).borrow_mut();

        let mut cnt = G_TIM5.borrow(cs).borrow_mut();
        let curr_ms = cnt.as_mut().unwrap().now().duration_since_epoch().to_millis();

        if button.as_mut().unwrap().is_high() {
            if curr_ms - PREV_MS.borrow(cs).get() > 50 {
            // hprintln!("Button pressed: debounging bug!").unwrap();
                G_DELAYMS
                    .borrow(cs)
                    .set(G_DELAYMS.borrow(cs).get() - 500_u32);
                if G_DELAYMS.borrow(cs).get() < 500_u32 {
                    G_DELAYMS.borrow(cs).set(2000_u32);
                }
                led_red.as_mut().unwrap().toggle();
                PREV_MS.borrow(cs).set(0);
                // restart timer
                cnt.as_mut().unwrap().cancel().unwrap();
                let _ = cnt.as_mut().unwrap().start(100.secs());
            }
            else {
                PREV_MS.borrow(cs).set(curr_ms);
            }
        }
        // Reset the timer
        let mut tim = G_TIM2.borrow(cs).borrow_mut();
        let _ = tim.as_mut().unwrap().wait();
    });
}