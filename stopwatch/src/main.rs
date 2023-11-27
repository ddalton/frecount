#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_timer_CountDown;
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::str;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use cortex_m_rt::entry;
use ssd1322_di::display;
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::prelude::SysctlExt;
use tm4c123x_hal::prelude::U32Ext;
use tm4c123x_hal::sysctl;
use tm4c123x_hal::tm4c123x::interrupt;
use tm4c123x_hal::tm4c123x::NVIC;
use tm4c123x_hal::{
    self as hal,
    time::Hertz,
    timer::{Event::TimeOut, Timer},
};
use {
    display_interface_spi::SPIInterface,
    embedded_graphics::{
        mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
        pixelcolor::Gray4,
        prelude::*,
        text::{Baseline, Text},
    },
};

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

struct Stopwatch {
    buffer: [u8; 8],
}

impl Stopwatch {
    fn to_str(&self) -> &str {
        str::from_utf8(&self.buffer).unwrap()
    }

    fn tick(&mut self) {
        self.buffer[7] += 1;

        // ripple carry
        if self.buffer[7] == 58 {
            self.buffer[7] = 48;

            self.buffer[6] += 1;
            if self.buffer[6] == 58 {
                self.buffer[6] = 48;

                self.buffer[4] += 1;
                if self.buffer[4] == 58 {
                    self.buffer[4] = 48;

                    self.buffer[3] += 1;
                    if self.buffer[3] == 54 {
                        self.buffer[3] = 48;

                        self.buffer[1] += 1;
                        if self.buffer[1] == 58 {
                            self.buffer[1] = 48;

                            self.buffer[0] += 1;
                            if self.buffer[0] == 54 {
                                self.buffer = [48, 48, 58, 48, 48, 46, 48, 48];
                            }
                        }
                    }
                }
            }
        }
    }
}

// Since the timer is configured as a 32-bit concatenated timer, it is sufficient to work with
// TIMER1A.
fn enable_timer1_interrupt() {
    unsafe {
        NVIC::unmask(tm4c123x_hal::tm4c123x::Interrupt::TIMER1A);
    }
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = hal::Peripherals::take().unwrap();

    // Wrap up the SYSCTL struct into an object with a higher-layer API
    let mut sc = p.SYSCTL.constrain();
    let mut porta = p.GPIO_PORTA.split(&sc.power_control);

    // Pick our oscillation settings
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    // Configure the PLL with those settings
    let clocks = sc.clock_setup.freeze();

    // Setup Display
    let mut res = porta.pa7.into_push_pull_output();
    let dc = porta.pa6.into_push_pull_output();
    let sclk = porta
        .pa2
        .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control);
    let mosi = porta
        .pa5
        .into_af_push_pull::<hal::gpio::AF2>(&mut porta.control);
    // This needs to be configured with an internal pull up as it is not used
    // by the display NHD-3.12-25664UCB2
    let miso = porta
        .pa4
        .into_af_pull_up::<hal::gpio::AF2>(&mut porta.control);

    let mut delay = Delay::new(cp.SYST, &clocks);
    let pins = (sclk, miso, mosi);

    let spi = hal::spi::Spi::spi0(
        p.SSI0,
        pins,
        hal::spi::MODE_0,
        2_u32.mhz(),
        &clocks,
        &sc.power_control,
    );

    let cs = porta.pa3.into_push_pull_output();
    let spi_interface = SPIInterface::new(spi, dc, cs);

    let mut disp = display::Ssd1322::new(spi_interface);

    // reset and init
    disp.reset(&mut res, &mut delay).unwrap();
    disp.init().unwrap();
    disp.clear(Gray4::new(0x00)).unwrap();
    disp.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Gray4::new(0b0000_1111))
        .background_color(Gray4::new(0b0000_0000))
        .build();

    // Stopwatch instance
    let mut sw = Stopwatch {
        buffer: [48, 48, 58, 48, 48, 46, 48, 48],
    };
    /*
    Text::with_baseline(sw.to_str(), Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();

    // Setup 100Hz timer
    enable_timer1_interrupt();
    let mut timer = Timer::timer1(p.TIMER1, Hertz(1), &sc.power_control, &clocks);
    timer.listen(TimeOut);
    */

    /*
    sw.tick();
    Text::with_baseline(sw.to_str(), Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut disp)
        .unwrap();
    disp.flush().unwrap();
    */

    loop {
        delay.delay_ms(10u16);
        //        if INTERRUPTED.load(Ordering::Relaxed) {
        //            INTERRUPTED.store(false, Ordering::SeqCst);

        Text::with_baseline(sw.to_str(), Point::new(10, 16), text_style, Baseline::Top)
            .draw(&mut disp)
            .unwrap();
        disp.flush_changed().unwrap();

        sw.tick();
        //        }
    }
}

#[interrupt]
fn TIMER1A() {
    INTERRUPTED.store(true, Ordering::SeqCst);
}
