## Hardware

The software in this repository orchestrates the following hardware:

- Camera to record fly behavior (JAI SP-5000M-CXP4 camera via Euresys Coaxlink Quad G3 frame grabber).
- IR LED to provide illumination for behavior recording (CCS LDR2-74IR2-850-LA ring light via CCS PD3-3024-3-EI(A) controller).
- Camera to record muscle activities (Excelitas pco.panda 4.2 camera with direct USB connection).
- Blue LED to provide excitation for calcium imaging (Thorlabs M470L3 LED via Thorlabs LEDD1B LED driver).
- Translation stages to move the optical system to follow the fly (two Zaber LSQ150A-E01CT3A translation stages via a single Zaber X-MCC2 controller).
- Two additional light control channels provided by the CCS PD3-3024-3-EI(A) controller for optogenetic stimulation. The controller allows for three channels, the first one being occupied by the IR LED. A vibrator can also be connected to the CCS controller in place of an LED to provide mechanical stimulation to the fly.
- Microcontroller to provide TTL signals to trigger cameras for frame capture and light controllers for strobing (Arduino Nano ESP32). The computer that the recorder software runs on sends commands to the triggering microcontroller via serial IO (USB).

The KiCad schematic and PCB layout for the trigger circuit board are in [`trigger_hardware/`](../../trigger_hardware/).

For discussion on camera frame rates, see [Frame rate limits](../users_manual/frame_rate_limits.md).