# `Building and running`

This project uses the evaluation board EK-TM4C123GXL to blink the led.

This is my first embedded project with rust. 
Some issues that were learnt the hard way is documented here and maybe useful for others running into the same issue.

The biggest issue I faced with developing this project was the implementation of the delay functionality.
I was using a loop that did nothing to create a delay. But the code was locking up my evaluation board and I had to use LMFlash utility to unlock it.

The root cause was that the rust compiler was removing the delay code and so the processor was turning on/off the RGB led light at very high speed.
This was eventually causing the evaluation board to lock up.

So the solution was to use the built in timer functionality to implement the delay.

## How the problem was identified

I produced an objdump of the code with changes made to the delay value and noticed that no changes were being made to the objdump. This made me realize the compiler was not producing this code at all.

``` console
$ cargo objdump --bin blink_interrupt --release -- --disassemble --no-show-raw-insn --print-imm-hex > out1
$ # Change the value of the delay and take the dump again
$ cargo objdump --bin blink_interrupt --release -- --disassemble --no-show-raw-insn --print-imm-hex > out2
$ # Take a diff of the two files
$ diff out1 out2
```

## Building

Run the following command to create a release build

``` console
$ cargo build --release
```

## Flash using gdb

On a terminal run openocd to connect to the debug MCU on the board. Run this command from the root of the template; openocd will pick up the openocd.cfg file which indicates which interface file and target file to use.
NOTE: the power switch should be towards the debug MCU.

``` console
$ openocd
```

On another terminal run GDB, also from the root of the template.

``` console
$ arm-none-eabi-gdb -q target/thumbv7em-none-eabihf/release/blink_interrupt
```

Next connect GDB to OpenOCD, which is waiting for a TCP connection on port 3333.

``` console
(gdb) target extended-remote :3333
```

Now proceed to flash (load) the program onto the microcontroller using the load command.

``` console
(gdb) load
```

At this point you can use the continue command or press the reset button to start running the program

``` console
(gdb) continue


# Miscellaneous

Timer initialization

```
use tm4c123x::TIMER0;
use tm4c123x_hal::timer::Timer;
use tm4c_hal::time::Hertz;

    unsafe {
        // 20ms delay
        // TODO: Use the Delay timer for 20ms debounce
        DEBOUNCE_TIMER = Some(Timer::<TIMER0>::timer0(
            p.TIMER0,
            Hertz(50),
            &sc.power_control,
            &clocks,
        ));
    }
```
