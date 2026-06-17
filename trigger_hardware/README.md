# Trigger circuit hardware

This folder contains the hardware design of the trigger controller.

See `TriggerCircuitBoard.pdf` for a schematic diagram of the triggering circuit. The CAD files (`.kicad_pro`, `.kicad_sch`, `.kicad_pcb`) can be opened using [KiCAD](https://www.kicad.org/).

## Bill of materials

Note: Symbols are as shown in the schematic diagram. All components are through-hole-mounted (THT), 2.54 mm pitch. Components can generally be replaced by equivalent parts. Exception: replacement of the microcontroller and the status display is possible but requires firmware code change.

| Symbol | Description | Model |
| --- | --- | --- |
| A1 | Microcontroller | Arduino Nano ESP32 |
| Q1–3 | NPN BJT transistors | Onsemi BC33725BU |
| R1–3 | ~500 Ω resistors | Vishay MBB02070C5600FCT00 |
| R4–6 | ~1 kΩ resistors | Vishay MRS16000C1001FC100 |
| D1 | RGB LED | RND 135-00191 |
| DISP1 | Display with I2C backpack | Midas MDOB128064WV-YBI |
| C1 | Polarized capacitor, ~200 μF, ≥ 24 V | Würth Elektronik 860010473011
| C2 | Non-polarized capacitor, ~100 nF, ≥ 3.3 V | Kemet μ1J63 1M2 |
| SW1 | Double pole, single throw (DPST) switch | RND 210-00549 |
| SW2 | Pushbutton | RND 210-00577 |
| J1–2 | Header pins & receptacles | Preci-Dip 890-18-036-10-804 / 3 (pins, soldered on Arduino) & 801-87-050-10-001101 (receptacles, soldered on PCB)
| J3 | 10-contact IDC connectors & cable | RS PRO 625-7252 plug (soldered on PCB), Würth Elektronik 63911015521CAB ribbon cable with RND 205-00682 socket (PCB side) and 3M 7100234235 panel-mount socket (on the exterior of the electronics box)
| J4 | 2-contact screw terminal & DC pwer connector | Phoenix Contact 2204260 screw terminal & RND 205-00909 external DC power connector |
| J5–10 | 4-contact screw terminal | Phoenix Contact 1725672 |
| J11–14 | BNC RF socket | RS PRO 546-3995 |

Additionally: Various jump wires, prototyping PCB (RS PRO 159-6322) cutout and header pins/receptacles for status display mounting, prototyping PCB cutout for GND bus.