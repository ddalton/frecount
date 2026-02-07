#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Gray4,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use ls7366::Ls7366;
use ssd1322_di::display::Ssd1322;
use tm4c123x_hal as hal;
use tm4c123x_hal::delay::Delay;
use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::prelude::SysctlExt;
use tm4c123x_hal::spi::Spi;
use tm4c123x_hal::sysctl;
use tm4c123x_hal::time::U32Ext;
use tm4c123x_hal::timer::{Event, Timer};
use tm4c123x_hal::tm4c123x::{self, interrupt, NVIC, TIMER0};

// --- Globals for timer ISR ---

static mut TIMER0_PERIPH: Option<Timer<TIMER0>> = None;

/// PD6 masked GPIO data address (APB, Port D, bit 6 only).
/// Base 0x4000_7000 + ((1 << 6) << 2) = 0x4000_7100
const PD6_DATA_ADDR: u32 = 0x4000_7100;
const PD6_BIT: u32 = 1 << 6;

// --- Helpers ---

/// Small buffer for formatting integers in no_std.
struct FmtBuf {
    buf: [u8; 20],
    pos: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 20],
            pos: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[..self.pos]).unwrap_or("???")
    }

    fn reset(&mut self) {
        self.pos = 0;
    }
}

impl Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if self.pos < self.buf.len() {
                self.buf[self.pos] = b;
                self.pos += 1;
            }
        }
        Ok(())
    }
}

/// Start the 1kHz signal generator (TIMER0).
fn start_signal_timer() {
    unsafe {
        let timer0 = &*tm4c123x::TIMER0::ptr();
        timer0.ctl.modify(|_, w| w.taen().set_bit());
    }
}

/// Stop the 1kHz signal generator (TIMER0).
fn stop_signal_timer() {
    unsafe {
        let timer0 = &*tm4c123x::TIMER0::ptr();
        timer0.ctl.modify(|_, w| w.taen().clear_bit());
    }
}

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = hal::Peripherals::take().unwrap();

    let mut sc = p.SYSCTL.constrain();

    // Configure PLL to 80 MHz
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    // --- GPIO port setup ---
    let porta = p.GPIO_PORTA.split(&sc.power_control);
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let mut portd = p.GPIO_PORTD.split(&sc.power_control);
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);

    // --- SSI2 SPI pins (Port B) ---
    let sclk = portb
        .pb4
        .into_af_push_pull::<hal::gpio::AF2>(&mut portb.control);
    let miso = portb
        .pb6
        .into_af_pull_up::<hal::gpio::AF2>(&mut portb.control);
    let mosi = portb
        .pb7
        .into_af_push_pull::<hal::gpio::AF2>(&mut portb.control);

    let spi = Spi::spi2(
        p.SSI2,
        (sclk, miso, mosi),
        hal::spi::MODE_0,
        1_u32.mhz(),
        &clocks,
        &sc.power_control,
    );

    // --- Counter pins ---
    let counter_cs = porta.pa3.into_push_pull_output(); // CS for LS7366
    let mut counter_b = porta.pa2.into_push_pull_output(); // B input (direction)
    counter_b.set_low().ok(); // LOW = count up
    let mut cnt_en = porte.pe4.into_push_pull_output(); // CNT_EN (active low)
    cnt_en.set_low().ok(); // LOW = counting enabled
    let _index = portb.pb2.into_floating_input(); // INDEX (unused, leave as input)
    let _dflag = porta.pa4.into_floating_input(); // DFLAG (unused, leave as input)

    // Counter A signal pin (PD6) - toggled by TIMER0 ISR
    let mut signal_a = portd.pd6.into_push_pull_output();
    signal_a.set_low().ok();

    // --- Display pins ---
    let display_cs = porte.pe0.into_push_pull_output();
    let display_dc = portf.pf4.into_push_pull_output();
    let pd7 = portd.pd7.unlock(&mut portd.control);
    let mut display_reset = pd7.into_push_pull_output();

    let mut delay = Delay::new(cp.SYST, &clocks);

    // --- Initialize display (one-time) ---
    let spi_iface = SPIInterface::new(spi, display_dc, display_cs);
    let mut disp = Ssd1322::new(spi_iface);

    disp.reset(&mut display_reset, &mut delay).unwrap();
    disp.init().unwrap();
    disp.clear(Gray4::new(0x00)).unwrap();
    disp.flush_all().unwrap();

    // Release display to get SPI back
    let spi_iface = disp.release();
    let (spi, mut display_dc, mut display_cs) = spi_iface.release();

    // --- Initialize counter (NonQuad mode) ---
    let mut counter = Ls7366::new_uninit(spi, counter_cs).unwrap();

    // MDR0: NonQuad (A=clock, B=direction), FreeRunning, no index
    let mdr0 = ls7366::mdr0::Mdr0 {
        quad_count_mode: ls7366::mdr0::QuadCountMode::NonQuad,
        cycle_count_mode: ls7366::mdr0::CycleCountMode::FreeRunning,
        index_mode: ls7366::mdr0::IndexMode::DisableIndex,
        is_index_inverted: false,
        filter_clock: ls7366::mdr0::FilterClockDivisionFactor::One,
    };
    counter
        .write_register(ls7366::Target::Mdr0, &[ls7366::Encodable::encode(&mdr0)])
        .unwrap();

    // MDR1: 4-byte counter, counting enabled, no flags
    let mdr1 = ls7366::mdr1::Mdr1 {
        counter_mode: ls7366::mdr1::CounterMode::Byte4,
        disable_counting: false,
        flag_on_idx: false,
        flag_on_cmp: false,
        flag_on_bw: false,
        flag_on_cy: false,
    };
    counter
        .write_register(ls7366::Target::Mdr1, &[ls7366::Encodable::encode(&mdr1)])
        .unwrap();

    counter.clear_status().unwrap();

    // --- Setup TIMER0 at 2kHz (toggles PD6 in ISR -> 1kHz square wave) ---
    let timer = Timer::timer0(p.TIMER0, 2000_u32.hz(), &sc.power_control, &clocks);
    unsafe {
        TIMER0_PERIPH = Some(timer);
        TIMER0_PERIPH.as_mut().unwrap().listen(Event::TimeOut);
    }
    unsafe {
        NVIC::unmask(tm4c123x::Interrupt::TIMER0A);
    }

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Gray4::new(0x0F))
        .background_color(Gray4::new(0x00))
        .build();

    let mut fmt_buf = FmtBuf::new();

    // --- Main loop ---
    loop {
        // Count pulses for 1 second
        delay.delay_ms(1000u32);

        // Stop the signal generator
        stop_signal_timer();

        // Read count value
        let count = counter.get_count().unwrap();

        // Release SPI from counter
        let (spi, counter_cs) = counter.free();

        // Build display interface and update display
        let spi_iface = SPIInterface::new(spi, display_dc, display_cs);
        let mut disp = Ssd1322::new(spi_iface);

        disp.clear(Gray4::new(0x00)).unwrap();

        Text::with_baseline("Counter:", Point::new(10, 4), text_style, Baseline::Top)
            .draw(&mut disp)
            .unwrap();

        fmt_buf.reset();
        write!(fmt_buf, "{}", count).unwrap();
        Text::with_baseline(
            fmt_buf.as_str(),
            Point::new(10, 30),
            text_style,
            Baseline::Top,
        )
        .draw(&mut disp)
        .unwrap();

        disp.flush_all().unwrap();

        // Release display, get SPI back
        let spi_iface = disp.release();
        let (spi, dc, cs) = spi_iface.release();
        display_dc = dc;
        display_cs = cs;

        // Recreate counter (chip config persists in hardware)
        counter = Ls7366::new_uninit(spi, counter_cs).unwrap();

        // Restart signal generator
        start_signal_timer();
    }
}

/// TIMER0A ISR: toggles PD6 to generate 1kHz square wave.
#[interrupt]
fn TIMER0A() {
    // Toggle PD6 via masked GPIO address (only affects bit 6)
    unsafe {
        let addr = PD6_DATA_ADDR as *mut u32;
        let current = core::ptr::read_volatile(addr);
        core::ptr::write_volatile(addr, current ^ PD6_BIT);
    }

    // Clear the timer interrupt flag
    unsafe {
        if let Some(ref mut timer) = TIMER0_PERIPH {
            let _ = timer.wait();
        }
    }
}
