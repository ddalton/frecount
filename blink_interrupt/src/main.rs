#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use tm4c123x::interrupt;
use tm4c123x::NVIC;
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::prelude::{SysctlExt, _embedded_hal_blocking_delay_DelayMs};
use tm4c123x_hal::sysctl;
use tm4c123x_hal::{
    self as hal,
    gpio::{
        gpiof::{Parts, PF0, PF1, PF2, PF3, PF4},
        Input, InterruptMode, Output, PullUp, PushPull,
    },
};

static DELAY: AtomicUsize = AtomicUsize::new(500);

// PF4 - Left Button
// PF0 - Right Button
//
// The switches tie the GPIO to ground, so the GPIOs need to be configured
// with pull-ups, and a value of 0 means the switch is pressed.
struct Rgb {
    red: PF1<Output<PushPull>>,
    blue: PF2<Output<PushPull>>,
    green: PF3<Output<PushPull>>,
    rb: PF0<Input<PullUp>>,
    lb: PF4<Input<PullUp>>,
}

impl Rgb {
    fn new(mut portf: Parts) -> Self {
        Rgb {
            red: portf.pf1.into_push_pull_output(),
            blue: portf.pf2.into_push_pull_output(),
            green: portf.pf3.into_push_pull_output(),
            rb: portf.pf0.unlock(&mut portf.control).into_pull_up_input(),
            lb: portf.pf4.into_pull_up_input(),
        }
    }

    fn set_low(&mut self) {
        self.red.set_low().unwrap();
        self.blue.set_low().unwrap();
        self.green.set_low().unwrap();
    }

    fn set_high(&mut self) {
        self.red.set_high().unwrap();
        self.blue.set_high().unwrap();
        self.green.set_high().unwrap();
    }

    fn enable_interrupts(&mut self) {
        self.rb.set_interrupt_mode(InterruptMode::EdgeRising);
        self.lb.set_interrupt_mode(InterruptMode::EdgeRising);
    }
}

#[interrupt]
fn GPIOF() {}

fn enable_gpio_interrupts() {
    unsafe {
        NVIC::unmask(tm4c123x::Interrupt::GPIOF);
    }
}

#[entry]
fn main() -> ! {
    let p = hal::Peripherals::take().unwrap();

    // Wrap up the SYSCTL struct into an object with a higher-layer API
    let mut sc = p.SYSCTL.constrain();
    // Pick our oscillation settings
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    // Configure the PLL with those settings
    let clocks = sc.clock_setup.freeze();

    let portf = p.GPIO_PORTF.split(&sc.power_control);
    let mut rgb = Rgb::new(portf);

    rgb.set_low();

    let mut on = false;
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut timer = Delay::new(cp.SYST, &clocks);

    enable_gpio_interrupts();
    rgb.enable_interrupts();

    loop {
        timer.delay_ms(DELAY.load(Ordering::SeqCst) as u32);

        // Toggle LED
        if on {
            rgb.set_high();
        } else {
            rgb.set_low();
        }
        on ^= true;
    }
}
