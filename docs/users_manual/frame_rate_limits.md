# Frame rate limits

Reference calculations for the maximum achievable frame rates of the two cameras.
See [Data acquisition](data_acquisition.md) for how the muscle camera's
continuous (auto-sequence) mode and the behavior/muscle synchronization work.

## Behavior camera

- Camera: JAI SP-5000M-CXP4
- Frame grabber: Euresys Coaxlink Quad G3
- Interface: CXP6 ×4

### Frame-rate calculation

Based on the manual [Manual_SP-5000-CXP4](https://www.jai.com/products/sp-5000m-cxp4/),
section 7.1.2 "Calculation of frame rate" (pages 32–33).

Frame readout time:

$$t_{\rm readout} = \frac{t_{\rm row} (N_{\rm row} + K + 2) + 1 + 3 t_{\rm row} + N_{\rm cycleF}}{f_{\rm sys}}.$$

Maximum frame rate:

$$f_{\rm max} = \frac{1}{t_{\rm readout}},$$

where

$$K = \left\lceil \frac{203}{t_{\rm row}} \right\rceil + 3,\qquad f_{\rm sys} = 86{,}400{,}000.$$

The following values depend on ROI width, bit format, and link type. For ROI width
1984, mono8, CXP6 ×4:

$$t_{\rm row} = H_{\rm blanking} = 130,\qquad N_{\rm cycleF} = 1022.$$

Therefore, for $N_{\rm row} = 1500$, $N_{\rm col} = 1984$:

$$K=\left\lceil\frac{203}{130}\right\rceil + 3 = 5,$$

$$
f_{\rm max}
= \frac{86{,}400{,}000}{130(1500 + 5 + 2) + 1 + 3\cdot 130 + 1022}
= 437.86\ {\rm Hz}.
$$

## Muscle camera

- Camera: pco.panda 4.2

### Frame-rate calculation

Based on the [pco.panda 4.2 manual](https://www.excelitas.com/product/pcopanda-42-scmos-camera), section 6 "Rolling
shutter" (pages 11–14).

The muscle camera runs in **continuous** rolling-shutter mode: each line begins
exposing the next frame as soon as *its own* readout of the previous frame
completes, even while lines below it are still exposing or being read out (the
"exposure time > sensor frame readout time" figure on page 12 of the manual). There
is no per-frame idle time. This is depicted in the sketch below.

<img width="500" alt="continuous rolling shutter timing" src="https://github.com/user-attachments/assets/ac59d1bc-7d2d-42f4-8c17-8721e81cd1ba" />

where (see also [Data acquisition](data_acquisition.md)):

- The thin, green lines are the exposure periods.
- The thick, red lines are the sensor readout periods.
- $t_{\rm so}$ is the shutter-open time (i.e., the exposure time communicated to the
  muscle camera).
    - Note: Since we're emulating a "global shutter" by turning on illumination only
      during the common time, this exposure time is *not* exposed to the
      higher-level API. As far as the caller is concerned, the exposure time is the
      duration during which the light is switched on.
- $t_{\rm lo}$ is the duration during which the light is switched on (i.e., the
  "exposure time" exposed to the caller, as noted above).
- $t_{\rm r}$ is the shutter rolling time (i.e., the amount of time between the start
  of exposure of the first line and that of the last line while acquiring the same
  frame).
- $\Delta t$ is the per-line readout time (i.e., for the camera to register the data
  and prepare for the next frame).
- $t_{\rm c}$ is the common time (i.e., the time during which all lines are exposed).
    - Note: In this continuous rolling mode, the only way to change the frame rate is
      to extend or reduce the common time. Therefore, to decouple frame-rate control
      from light-on time control, we can add a buffer time $t_{\rm b}$ during which
      all lines are exposed, but the light is off.
- $t_{\rm b}$ is the buffer time as described above.
- $T$ is the recording interval (i.e., 1 / frame rate).

From the manual:

- Line time = $12.136\ {\rm \mu s}$. At 1120×1120 px,
  $t_{\rm r} = 1120\cdot 12.136 = 13592.32\ {\rm \mu s}$.
- $\Delta t = 4.5\ {\rm \mu s}$.

Frame rate:

$$
f = \frac{10^6}{t_{\rm r} + t_{\rm lo} + t_{\rm b} + \Delta t}
= \frac{10^6}{13596.82 + t_{\rm lo} + t_{\rm b}}\ {\rm Hz},
$$

$$
t_{\rm b} = \frac{10^6}{f} - t_{\rm r} - t_{\rm lo} - \Delta t.
$$

The maximum frame rate is reached when $t_{\rm b} = 0$:

$$
f_{\rm max} = \frac{10^6}{13596.82 + t_{\rm lo}}\ {\rm Hz}.
$$

At 1 ms effective exposure, $f_{\rm max} = 68.5\ {\rm Hz}$.

## A reasonable operating point

- Behavior: 1500×1984 px at 396 Hz.
- Muscle: 1120×1120 px at 66 Hz.
- Behavior/muscle synchronization ratio: 6.
- This requires $t_{\rm b} = 10^6/66 - 13592.32 - 1000 - 4.5 = 554.695\ {\rm \mu s}$.
- Empirically ~15 GiB of data per minute (~11.3 GiB behavior + ~3.6 GiB muscle).
