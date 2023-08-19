# Variable speed blinking

This project uses the evaluation board EK-TM4C123GXL to blink the led.

This project builds upon the previous project blink1 and introduces the concept of interrupts.
The design here is that the delay of the blinking is controlled by the 2 buttons at the button of the board.
The right button that is below the LED, reduces the delay thus increasing blink speed and the left button does the opposite.

One critical issue with the interrupt code is that once an interrupt is handled any subsequent interrupts need to be cleared. This can happen due to bouncing of the switch.
openocd and gdb is critical to troubleshoot any issues here.

The following gdb commands were helpful to debug one issue where the clearing of the interrupts were necessary:
```
(gdb) target extended-remote :3333

list main
break 172
print rgb.delay_cnt

// Step over 20 instructions in enable and disable interrupt methods
step 20
```
