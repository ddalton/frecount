#![no_std]
#![no_main]

use hal::time::U32Ext;
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use ssd1322_di::display;
use tm4c123x_hal as hal;
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::prelude::SysctlExt;
use tm4c123x_hal::sysctl;
use {
    display_interface_spi::SPIInterface,
    embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::Gray4,
        prelude::*,
        text::{Baseline, Text},
    },
};

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
        .font(&FONT_6X10)
        .text_color(Gray4::new(0b0000_1111))
        .build();

    Text::with_baseline("Hello World!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut disp)
        .unwrap();

    Text::with_baseline(
        "by yours truly.",
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut disp)
    .unwrap();

    disp.flush().unwrap();

    loop {}
}
