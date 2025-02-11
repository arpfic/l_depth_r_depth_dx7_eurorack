# L&R Depth

This folder contains the hardware design and software for a **Left & Right (L&R) Depth module** with a breakpoint/plateau, inspired by the famous DX7 function from Yamaha synthesizers. The goal was to provide a eurorack module that can shape control voltages or signals with separate left/right “depth” ranges, plus an adjustable center plateau zone.

More photos coming soon !

## Hardware

- **Eurorack** (+12/-12V) format (see the [`Hardware`](Hardware/) folder).
- Exposes:
  - **CV input** for the main signal.
  - Pots/sliders for left/right depth and a central “plateau” adjustment.
  - Digital switches/buttons to toggle between **linear** and **exponential** shapes.
- Panel:

![LRDepth_pannel](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/Hardware/lr_pannel.jpg)

- PCB:

![LRDepth_top](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/Hardware/lr_depth_top.jpg)
![LRDepth_bottom](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/Hardware/lr_depth_bottom.jpg)

## Software

- Compiled under Mbed using a custom profile (e.g. -O3 optimizations) provided in this repository.

## Example Plots

Here are some example curves generated by the firmware, showing both **linear** and **exponential** transitions:

![Linearcurve](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/lin_final.png)  
![Expcurve](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/exp_w_c=4_final.png)  
![Linearother](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/lin_other.png)  
![Expother](https://raw.githubusercontent.com/arpfic/l_depth_r_depth_dx7_eurorack/master/exp_other.png)

## Notes, Limitations, and futures Improvements:

- **Re-implement center control so it affects only the center plateau** (reducing it without altering the side breakpoints), and apply inverted curves on the side transitions. 
- Mbed: It would be beneficial to migrate away from Mbed toward a lower-level library such as libopencm3 for more direct control.
- ADC (12-bit): Consider using an external ADC with higher resolution—likewise for the DAC—to achieve finer precision.
- Current Loop Speed (~10 kHz): The calculation loop runs at about 10,000 iterations per second, which can limit experimentation if you want to process signals beyond simple CV. To significantly increase throughput, implementing DMA on the ADC inputs would be essential.

---

## DX7 Reference

- On the Yamaha DX7, “L & R Depth” parameters control how certain amplitude or modulation segments transition on the left or right side of a breakpoint. This module adapts that concept for a modern Eurorack environment, giving analog control over a similarly flexible depth function.

---

Principal code maintainer : Damien Leblois <damienleb@protonmail.com>
