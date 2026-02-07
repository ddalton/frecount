#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal::spi::{Error, ErrorKind, ErrorType as SpiErrorType, SpiBus, SpiDevice};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use core::str;
use core::sync::atomic::{AtomicBool, Ordering};
use ssd1322_di::display;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Gray4,
    prelude::*,
    text::{Baseline, Text},
};
use tm4c123x_hal::{
    self as hal,
    delay::Delay,
    gpio::GpioExt,
    prelude::*,
    spi::{Spi, MODE_0},
    sysctl::{self, SysctlExt},
    time::Hertz,
    timer::{Event, Timer},
    tm4c123x::{interrupt, NVIC, SSI2, TIMER1},
};

// Global timer for interrupt
static INTERRUPTED: AtomicBool = AtomicBool::new(false);
static mut TIMER_100HZ: Option<TimerWrapper> = None;

// Stopwatch state
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

// SPI wrapper to adapt tm4c123x_hal::spi::Spi to embedded_hal::spi::SpiBus
struct SpiBusWrapper {
    spi: Spi<SSI2, (hal::gpio::Pin<hal::gpio::portb::PB4, hal::gpio::AlternateFunction<hal::gpio::PushPull, 2>>, hal::gpio::Pin<hal::gpio::portb::PB6, hal::gpio::AlternateFunction<hal::gpio::PullUp, 2>>, hal::gpio::Pin<hal::gpio::portb::PB7, hal::gpio::AlternateFunction<hal::gpio::PushPull, 2>>)>,
}

impl SpiBusWrapper {
    fn new(spi: Spi<SSI2, (hal::gpio::Pin<hal::gpio::portb::PB4, hal::gpio::AlternateFunction<hal::gpio::PushPull, 2>>, hal::gpio::Pin<hal::gpio::portb::PB6, hal::gpio::AlternateFunction<hal::gpio::PullUp, 2>>, hal::gpio::Pin<hal::gpio::portb::PB7, hal::gpio::AlternateFunction<hal::gpio::PushPull, 2>>)>) -> Self {
        SpiBusWrapper { spi }
    }
}

#[derive(Debug)]
pub enum SpiError {
    Overrun,
    Timeout,
    FifoFull,
    FifoEmpty,
}

impl Error for SpiError {
    fn kind(&self) -> ErrorKind {
        match self {
            SpiError::Overrun => ErrorKind::Overrun,
            SpiError::Timeout => ErrorKind::Other,
            SpiError::FifoFull => ErrorKind::Other,
            SpiError::FifoEmpty => ErrorKind::Other,
        }
    }
}

impl SpiErrorType for SpiBusWrapper {
    type Error = SpiError;
}

impl SpiBus<u8> for SpiBusWrapper {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut dummy = vec![0u8; words.len()];
        self.spi.transfer(words, &mut dummy).map_err(|_| SpiError::Timeout)?;
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write(words).map_err(|_| SpiError::Timeout)?;
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        if read.len() != write.len() {
            return Err(SpiError::Timeout);
        }
        self.spi.transfer(read, write).map_err(|_| SpiError::Timeout)?;
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut dummy Hawkins for word in words.iter_mut() {
            let dummy = vec![0u8; words.len()];
            self.spi.transfer(words, &dummy).map_err(|_| SpiError::Timeout)?;
            Ok(())
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        while self.spi.ssi.sr.read().bsy().bit_is_set() {}
        Ok(())
    }
}

// Timer wrapper to adapt tm4c123x_hal::timer::Timer for interrupt-driven timing
struct TimerWrapper {
    timer: Timer<TIMER1>,
}

impl TimerWrapper {
    fn new(timer: Timer<TIMER1>) -> Self {
        TimerWrapper { timer }
    }

    fn listen(&mut self) {
        self.timer.listen(Event::TimeOut);
    }

    fn wait(&mut self) {
        self.timer.wait();
    }
}

// Custom error type for infallible operations
#[derive(Debug)]
pub enum NoError {}

impl ErrorType for PinWrapper<PIN>
where
    PIN: hal::gpio::OutputPin,
{
    type Error = NoError;
}

impl<PIN> embedded_hal::digital::OutputPin for PinWrapper<PIN>
where
    PIN: hal::gpio::OutputPin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }
}

// Delay wrapper to adapt tm4c123x_hal::delay::Delay to embedded_hal::delay::DelayNs
struct DelayWrapper {
    delay: Delay,
}

impl DelayWrapper {
    fn new(delay: Delay) -> Self {
        DelayWrapper { delay }
    }
}

impl DelayNs for DelayWrapper {
    fn delay_ns(&mut self, ns: u32) {
        let ms = (ns + 999_999) / 1_000_000;
        self.delay.delay_ms(ms);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay.delay_ms(ms);
    }

    fn delay_us(&mut self, us: u32) {
        let ms = (us + 999) / 1000;
        self.delay.delay_ms(ms);
    }
}

// Enable timer interrupt
fn enable_timer1_interrupt() {
    unsafe {
        NVIC::unmask(hal::tm4c123x::Interrupt::TIMER1A);
    }
}

#[entry]
fn main() -> ! {
    let p = hal::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Initialize clocks
    let mut sc = p.SYSCTL.constrain();
    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    // Initialize GPIO
    let porte = p.GPIO_PORTE.split(&sc.power_control);
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
    let portf = p.GPIO_PORTF.split(&sc.power_control);
    let mut portd = p.GPIO_PORTD.split(&sc.power_control);
    let pd7 = portd.pd7.unlock(&mut portd.control);

    // SPI pins (SSI2)
    let sclk = portb
        .pb4
        .into_af_push_pull::<hal::gpio::AF2>(&mut portb.control);
    let mosi = portb
        .pb7
        .into_af_push_pull::<hal::gpio::AF2>(&mut portb.control);
    let miso = portb
        .pb6
        .into_af_pull_up::<hal::gpio::AF2>(&mut portb.control);
    let pins = (sclk, miso, mosi);

    // Initialize SPI
    let spi = Spi::spi2(
        p.SSI2,
        pins,
        MODE_0,
        2_000_000_u32.hz(),
        &clocks,
        &sc.power_control,
    );
    let spi_bus = SpiBusWrapper::new(spi);

    // GPIO pins
    let cs1 = PinWrapper::new(porte.pe0.into_push_pull_output());
    let cs2 = PinWrapper::new(porte.pe1.into_push_pull_output()); // Second device CS
    let dc = PinWrapper::new(portf.pf4.into_push_pull_output());
    let reset = PinWrapper::new(pd7.into_push_pull_output());

    // Initialize delay
    let mut delay = DelayWrapper::new(Delay::new(cp.SYST, &clocks));

    // Initialize timer
    let mut timer = Timer::timer1(p.TIMER1, Hertz(100), &sc.power_control, &clocks);
    let mut timer_wrapper = TimerWrapper::new(timer);

    // Create SPI devices sharing SSI2
    let display1 = ExclusiveDevice::new(spi_bus, cs1, &mut delay).unwrap();
    let display2 = ExclusiveDevice::new(display1.bus(), cs2, &mut delay).unwrap();

    // Setup display
    let spi_interface = SPIInterface::new(display1, dc);
    let mut disp = display::Ssd1322::new(spi_interface);

    // Reset and initialize display
    disp.reset(&mut reset, &mut delay).unwrap();
    disp.init().unwrap();
    disp.clear(Gray4::new(0x00)).unwrap();
    disp.flush_all().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Gray4::new(0b0000_1111))
        .background_color(Gray4::new(0b0000_0000))
        .build();

    // Setup 100Hz timer
    unsafe {
        TIMER_100HZ = Some(timer_wrapper);
        TIMER_100HZ.as_mut().map(|t| {
            t.listen();
        });
    }
    enable_timer1_interrupt();

    // Stopwatch instance
    let mut sw = Stopwatch {
        buffer: [48, 48, 58, 48, 48, 46, 48, 48],
    };

    loop {
        if INTERRUPTED.load(Ordering::Relaxed) {
            INTERRUPTED.store(false, Ordering::SeqCst);

            Text::with_baseline(sw.to_str(), Point::new(10, 16), text_style, Baseline::Top)
                .draw(&mut disp)
                .unwrap();
            disp.flush().unwrap();

            sw.tick();
        }
    }
}

#[interrupt]
fn TIMER1A() {
    INTERRUPTED.store(true, Ordering::SeqCst);
    unsafe {
        TIMER_100HZ.as_mut().map(|t| t.wait());
    }
}
