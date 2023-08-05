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
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::prelude::{SysctlExt, _embedded_hal_blocking_delay_DelayMs};
use tm4c123x_hal::sysctl;
use tm4c123x_hal::tm4c123x::interrupt;
use tm4c123x_hal::tm4c123x::NVIC;
use tm4c123x_hal::{
    self as hal,
    gpio::{
        gpiof::{Parts, PF0, PF1, PF2, PF3, PF4},
        Input, InterruptMode, Output, PullUp, PushPull,
    },
};

const DELAY: u16 = 1_u16;
static INTERRUPTED: AtomicBool = AtomicBool::new(false);
static mut BUTTONS: Option<Buttons> = None;

struct Rgb {
    red: PF1<Output<PushPull>>,
    blue: PF2<Output<PushPull>>,
    green: PF3<Output<PushPull>>,
    on: bool,
    delay: Delay,
    delay_cnt: u16,
    delay_idx: u16,
}

// PF4 - Left Button
// PF0 - Right Button
//
// The switches tie the GPIO to ground, so the GPIOs need to be configured
// with pull-ups, and a value of 0 means the switch is pressed.
struct Buttons {
    rb: PF0<Input<PullUp>>,
    lb: PF4<Input<PullUp>>,
}

impl Rgb {
    fn new(mut portf: Parts, delay: Delay) -> (Self, Buttons) {
        (
            Rgb {
                red: portf.pf1.into_push_pull_output(),
                blue: portf.pf2.into_push_pull_output(),
                green: portf.pf3.into_push_pull_output(),
                on: false,
                delay,
                delay_cnt: 1000 / DELAY,
                delay_idx: 0,
            },
            Buttons {
                rb: portf.pf0.unlock(&mut portf.control).into_pull_up_input(),
                lb: portf.pf4.into_pull_up_input(),
            },
        )
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

    fn toggle(&mut self) {
        // Toggle LED
        if self.on {
            self.set_high();
        } else {
            self.set_low();
        }
        self.on ^= true;
    }

    fn reset(&mut self) {
        self.delay_idx = 0;
        INTERRUPTED.store(false, Ordering::SeqCst);
    }

    fn process(&mut self) {
        unsafe {
            BUTTONS.as_mut().unwrap().handle_interrupt(self);
        }

        self.delay_idx += 1;
        if self.delay_idx % self.delay_cnt == 0 {
            self.toggle();
        }
    }
}

impl Buttons {
    fn enable_interrupts(&mut self) {
        // Since the buttons are active low, we have to trigger on falling edge
        self.rb.set_interrupt_mode(InterruptMode::EdgeFalling);
        self.lb.set_interrupt_mode(InterruptMode::EdgeFalling);
    }

    fn disable_interrupts(&mut self) {
        self.rb.set_interrupt_mode(InterruptMode::Disabled);
        self.lb.set_interrupt_mode(InterruptMode::Disabled);
        self.rb.clear_interrupt();
        self.lb.clear_interrupt();
    }

    fn handle_interrupt(&mut self, rgb: &mut Rgb) {
        if INTERRUPTED.load(Ordering::Relaxed) {
            rgb.reset();
            // Simple switch debouncing using a delay
            rgb.delay.delay_ms(20_u8);
            if self.lb.is_low().unwrap() {
                rgb.delay_cnt = min(2000 / DELAY, rgb.delay_cnt + 200 / DELAY);
            } else if self.rb.is_low().unwrap() {
                rgb.delay_cnt = max(200 / DELAY, rgb.delay_cnt - 200 / DELAY);
            }

            self.enable_interrupts();
        }
    }
}

fn enable_gpio_interrupts() {
    unsafe {
        NVIC::unmask(tm4c123x_hal::tm4c123x::Interrupt::GPIOF);
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
    let cp = cortex_m::Peripherals::take().unwrap();
    let timer = Delay::new(cp.SYST, &clocks);

    enable_gpio_interrupts();

    let portf = p.GPIO_PORTF.split(&sc.power_control);
    let (mut rgb, buttons) = Rgb::new(portf, timer);
    unsafe {
        BUTTONS.replace(buttons);
        BUTTONS.as_mut().unwrap().enable_interrupts();
    }

    loop {
        rgb.process();

        rgb.delay.delay_ms(DELAY);
    }
}

#[interrupt]
fn GPIOF() {
    // Disable interrupts
    unsafe {
        BUTTONS.as_mut().unwrap().disable_interrupts();
    }

    INTERRUPTED.store(true, Ordering::SeqCst);
}
