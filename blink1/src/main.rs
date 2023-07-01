#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::prelude::{SysctlExt, _embedded_hal_blocking_delay_DelayMs};
use tm4c123x_hal::sysctl;
use tm4c123x_hal::{
    self as hal,
    gpio::{
        gpiof::{Parts, PF1, PF2, PF3},
        Output, PushPull,
    },
};

// PF4 - Left Button
// PF0 - Right Button
//
// The switches tie the GPIO to ground, so the GPIOs need to be configured
// with pull-ups, and a value of 0 means the switch is pressed.

struct Rgb {
    red: PF1<Output<PushPull>>,
    blue: PF2<Output<PushPull>>,
    green: PF3<Output<PushPull>>,
    delay: u32,
}

impl Rgb {
    fn new(portf: Parts) -> Self {
        Rgb {
            red: portf.pf1.into_push_pull_output(),
            blue: portf.pf2.into_push_pull_output(),
            green: portf.pf3.into_push_pull_output(),
            delay: 500_u32,
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
    loop {
        timer.delay_ms(rgb.delay);

        // Toggle LED
        if on {
            rgb.set_high();
        } else {
            rgb.set_low();
        }
        on ^= true;
    }
}
