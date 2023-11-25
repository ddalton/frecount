#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m_rt::entry;
use tm4c123x_hal::prelude::SysctlExt;
use tm4c123x_hal::sysctl;
use tm4c123x_hal::tm4c123x::interrupt;
use tm4c123x_hal::tm4c123x::NVIC;
use tm4c123x_hal::{self as hal, time::Hertz, timer::Timer};

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

// Since the timer is configured as a 32-bit concatenated timer, it is sufficient to work with
// TIMER1A.
fn enable_timer1_interrupt() {
    unsafe {
        NVIC::unmask(tm4c123x_hal::tm4c123x::Interrupt::TIMER1A);
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
    let timer = Timer::timer1(p.TIMER1, Hertz(100), &sc.power_control, &clocks);

    enable_timer1_interrupt();

    loop {}
}

#[interrupt]
fn TIMER1A() {
    // Disable interrupts
    unsafe {
        //   BUTTONS.as_mut().unwrap().disable_interrupts();
    }

    INTERRUPTED.store(true, Ordering::SeqCst);
}
