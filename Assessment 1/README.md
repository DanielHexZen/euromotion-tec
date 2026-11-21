# Assessment 1 – LS / ARX identification (EuroMotion)

Due: Fri 20.11.2026, 23:59. Paper in the IEEE conference format, max. 8 pages.

## Steps

1. **Firmware:** copy `firmware/sysid.c` to `Core/Src` and `firmware/sysid.h` to `Core/Inc`. Make the 4 edits from `firmware/INTEGRATION.md`. Flash.
2. **Tests (car on the floor, USB cable, battery charged):** in MATLAB, in the `matlab` folder:
   ```matlab
   capture_sysid("COM5", 1, "data/lon_step_1.csv")
   capture_sysid("COM5", 2, "data/lon_prbs_1.csv")
   capture_sysid("COM5", 3, "data/lon_chirp_1.csv")
   capture_sysid("COM5", 4, "data/lat_prbs_1.csv")
   capture_sysid("COM5", 5, "data/lat_dlc_1.csv")
   capture_sysid("COM5", 6, "data/lat_slc_1.csv")
   ```
   Then all six again with `_2`. Use the right COM port.
3. **Identification:** run `matlab/sysid_A1.m`. It writes `results.tex` and the `fig_lon_*` / `fig_lat_*` PDFs into this folder.
4. **Photo:** a close-up of the car as `fig_car.jpg` in this folder.
5. **Paper:** compile `Assessment-1_A01832776.tex`. Fill in the red `[TODO: ...]` parts. Do not submit while red text is left.

The paper compiles before step 3, too. Missing numbers show as red `??`, missing figures as framed boxes.
