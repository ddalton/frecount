#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::cmp::{max, min};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use tm4c123x::NVIC;
use tm4c123x::{interrupt, pwm0::INTEN};
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

static mut RGB: Option<Rgb> = None;

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
    on: bool,
    delay: Delay,
    delay_cnt: i8,
    delay_idx: i8,
    interrupted: bool,
}

impl Rgb {
    fn new(mut portf: Parts, delay: Delay) -> Self {
        Rgb {
            red: portf.pf1.into_push_pull_output(),
            blue: portf.pf2.into_push_pull_output(),
            green: portf.pf3.into_push_pull_output(),
            rb: portf.pf0.unlock(&mut portf.control).into_pull_up_input(),
            lb: portf.pf4.into_pull_up_input(),
            on: false,
            delay,
            delay_cnt: 50,
            delay_idx: 0,
            interrupted: false,
        }
    }

    unsafe fn set_low(&mut self) {
        self.red.set_low().unwrap();
        self.blue.set_low().unwrap();
        self.green.set_low().unwrap();
    }

    unsafe fn set_high(&mut self) {
        self.red.set_high().unwrap();
        self.blue.set_high().unwrap();
        self.green.set_high().unwrap();
    }

    unsafe fn handle_interrupt(&mut self) {
        if self.interrupted {
            self.disable_interrupts();
            self.delay.delay_ms(20_u8);
            if self.lb.is_low().unwrap() {
                self.delay_cnt = min(100, self.delay_cnt + 10);
            } else if self.rb.is_low().unwrap() {
                self.delay_cnt = max(10, self.delay_cnt - 10);
            }

            self.enable_interrupts();
        }

        self.delay_idx += 1;
        if self.delay_idx % self.delay_cnt == 0 {
            self.toggle();
        }
    }

    unsafe fn interrupt(&mut self) {
        self.interrupted = true;
        self.disable_interrupts();
    }

    unsafe fn enable_interrupts(&mut self) {
        // Since the buttons are active low, we have to trigger on falling edge
        self.rb.set_interrupt_mode(InterruptMode::EdgeFalling);
        self.lb.set_interrupt_mode(InterruptMode::EdgeFalling);
        self.interrupted = false;
    }

    unsafe fn disable_interrupts(&mut self) {
        self.rb.set_interrupt_mode(InterruptMode::Disabled);
        self.lb.set_interrupt_mode(InterruptMode::Disabled);
        self.delay_idx = 0;
    }

    unsafe fn toggle(&mut self) {
        // Toggle LED
        if self.on {
            self.set_high();
        } else {
            self.set_low();
        }
        self.on ^= true;
    }
}

#[interrupt]
fn GPIOF() {
    unsafe {
        RGB.as_mut().unwrap().interrupt();
    }
}

fn enable_gpio_interrupts() {
    unsafe {
        NVIC::unmask(tm4c123x::Interrupt::GPIOF);
    }
}

#[entry]
unsafe fn main() -> ! {
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
    let cp = cortex_m::Peripherals::take().unwrap();
    let timer = Delay::new(cp.SYST, &clocks);

    let portf = p.GPIO_PORTF.split(&sc.power_control);
    RGB = Some(Rgb::new(portf, timer));
    // unwrap is expected to work
    RGB.as_mut().unwrap().set_low();

    enable_gpio_interrupts();
    RGB.as_mut().unwrap().enable_interrupts();

    loop {
        RGB.as_mut().unwrap().handle_interrupt();
    }
}
