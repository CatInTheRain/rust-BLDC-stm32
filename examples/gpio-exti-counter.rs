#![no_std]
#![no_main]

// IMports
use core::{cell::{Cell, RefCell}, convert::TryInto};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use cortex_m::peripheral::NVIC;
use cortex_m_semihosting::hprint;
use panic_semihosting as _;
// use panic_halt as _; //more optimized!
// use cortex_m_semihosting::hprintln;
use stm32f7xx_hal::{
    gpio::{self, Edge, Input, ExtiPin},
    pac::{self, interrupt, EXTI, TIM5},
    prelude::*,
    timer::CounterUs,
};

// Create an alias for pin PC13
// According to the Nucleo-144 Documentation:
// B1 USER: the user button is connected to the I/O PC13 by default [...]
type ButtonPin = gpio::PC13<Input>;
type LedButton = gpio::Pin<'B', 14, gpio::Output>;

// Global Variable Definitions
// Global variables are wrapped in safe abstractions.
// Peripherals are wrapped in a different manner than regular global mutable data.
// In the case of peripherals we must be sure only one reference exists at a time.
// Refer to Chapter 6 of the Embedded Rust Book for more detail.

// Create a Global Variable for the GPIO Peripheral that I'm going to pass around.
// RefCell is used to be able obtain a mutable reference to the peripheraL.
// Compared to a Box, RefCell allows for checking during runtime that only one mutable reference exists to a variable
static G_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));
static G_EXTI: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));
static G_TIM5: Mutex<RefCell<Option<CounterUs<TIM5>>>> = Mutex::new(RefCell::new(None));

// Create a Global Variable for the delay value that I'm going to pass around for delay.
// I am not using an Option since I am directly initializing with a value.
// Cell only permits taking a copy of the current value or replacing it
static G_DELAYMS: Mutex<Cell<u32>> = Mutex::new(Cell::new(2000_u32));
static G_START_CNT: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

static G_LED_RED: Mutex<RefCell<Option<LedButton>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    /* Parameters */ 
    let sysclk_freq: u32 = 216; // MHz

    // Setup handler for device peripherals: take the PAC level device peripherals
    let mut dp = pac::Peripherals::take().unwrap();
    // Configure and obtain handle for delay abstraction
    // 1) Promote RCC structure to HAL to be able to configure clocks
    let mut rcc = dp.RCC.constrain();
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
    let mut button = gpioc.pc13.into_floating_input();

    // Configure Button Pin for Interrupts
    // 1) Make button an interrupt source
    button.make_interrupt_source(&mut dp.SYSCFG, &mut rcc.apb2);
    // 2) Trigger only on the rising edge
    let mut exti = dp.EXTI;
    button.trigger_on_edge(&mut exti, Edge::Rising);
    // 3) Enable GPIO interrupt for button
    button.enable_interrupt(&mut exti);

    cortex_m::interrupt::free(|cs| {
        G_EXTI.borrow(cs).replace(Some(exti));
    });

    // Enable the external interrupt in the NVIC by passing the button interrupt number
    unsafe {
        NVIC::unmask::<interrupt>(interrupt::EXTI15_10);
    }

    // Now that button is configured, move button into global context
    // G_BUTTON was initialized with None pending the configuration of the GPIO button that is now available
    // Here we are introducing a critical section of code enclosed in the closure cortex_m::interrupt::free
    // The closure passes a token cs that allows us to borrow a mutable reference the global variable and replace
    // the Option inside of with Some(button)
    cortex_m::interrupt::free(|cs| {
        G_BUTTON.borrow(cs).replace(Some(button));
    });

    // Create a Millisecond Counter Handle
    let counter = dp.TIM5.counter_us(&clocks);  
    // Move the timer into our global storage
    cortex_m::interrupt::free(|cs| {
        G_TIM5.borrow(cs).replace(Some(counter));
    });

    // cortex_m::interrupt::free(|cs| {
    //     let mut cnt = G_TIM5.borrow(cs).borrow_mut();
    //     let _ = cnt.as_mut().unwrap().start(100.secs());
    // });

    // Application Loop
    loop{
        cortex_m::interrupt::free(|cs| {
            let mut cnt = G_TIM5.borrow(cs).borrow_mut();
            if G_START_CNT.borrow(cs).get() {
                let mut elapsed_time = cnt.as_mut().unwrap().now().duration_since_epoch().to_micros();
                let mut button = G_BUTTON.borrow(cs).borrow_mut();
                if elapsed_time > 150000 {
                    button.as_mut().unwrap().enable_interrupt(G_EXTI.borrow(cs).borrow_mut().as_mut().unwrap());
                    let err = cnt.as_mut().unwrap().cancel();
                    elapsed_time = 0;
                    G_START_CNT.borrow(cs).set(false);
                }
            }             
        });
    }
}

#[interrupt]
fn EXTI15_10() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain Access to Delay Global Data and Adjust Delay
        G_DELAYMS
            .borrow(cs)
            .set(G_DELAYMS.borrow(cs).get() - 500_u32);
        if G_DELAYMS.borrow(cs).get() < 500_u32 {
            G_DELAYMS.borrow(cs).set(2000_u32);
        }
        
        let mut led_red = G_LED_RED.borrow(cs).borrow_mut();
        led_red.as_mut().unwrap().toggle();
        // Obtain access to Global Button Peripheral and Clear Interrupt Pending Flag
        // The following line obtains a mutable reference to the Option in G_BUTTON using the borrow_mut method
        let mut button = G_BUTTON.borrow(cs).borrow_mut();
        // Finally, the clear_interrupt_pending_bit method from the ExtiPin traits 
        // is applied to clear the interrupt pending flag/bit
        button.as_mut().unwrap().clear_interrupt_pending_bit();
        button.as_mut().unwrap().disable_interrupt(G_EXTI.borrow(cs).borrow_mut().as_mut().unwrap());
        let mut cnt = G_TIM5.borrow(cs).borrow_mut();
        // cnt.as_mut().unwrap().cancel().unwrap();
        let _ = cnt.as_mut().unwrap().start(100.secs());
        G_START_CNT.borrow(cs).set(true);
    });
}