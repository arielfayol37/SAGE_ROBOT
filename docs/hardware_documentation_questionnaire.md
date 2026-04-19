# SAGE Hardware Documentation Questionnaire

**For: Mechanical & Electrical Engineers**

This document contains the questions you need to answer to produce complete hardware documentation for SAGE. The goal is that someone with no prior exposure to the robot — a new student, a faculty member, or an AI assistant — can read the resulting document and understand exactly how SAGE is physically built, how its electronics are connected, and how to diagnose or repair any hardware problem.

Please answer every question as precisely as possible. Include photos, diagrams, datasheets, or CAD screenshots wherever they help. Vague answers like "standard motor" or "generic ESC" are not sufficient — include part numbers, model names, and manufacturer names.

---

## Part 1: Chassis & Mechanical Structure

### 1.1 Frame Overview

1. What material(s) are the 3D-printed parts made from (PLA, PETG, ABS, etc.)? Were any parts printed with special infill settings or wall thickness for strength?
2. What is the overall robot footprint (length × width) and total height from the ground to the top of the lid? Include the height with the LIDAR mounted.
3. How many distinct 3D-printed assemblies are there (e.g., base platform, middle frame, lid)? Give each a name and briefly describe its function.
4. Where are the CAD/STL files stored? Provide the file path or repository link for each printed part.
5. What 3D printer(s) were used, and at what layer height and infill percentage were the parts printed?
6. Are there any non-printed structural parts (aluminum extrusion, sheet metal brackets, standoffs)? If yes, describe each and provide dimensions or part numbers.
7. How are the major assemblies (base, frame, lid) connected to each other? Screws (size and length), snap fits, bolts and nuts?
8. How is the Jetson Orin Nano mounted inside the robot? On what surface, with what fasteners, and is there any vibration isolation?
9. How is the STM32 board mounted? Where inside the robot is it located?

### 1.2 Wheels & Drive System

10. What are the two **driven wheels** (rear, differential drive)? Provide diameter, width, material (rubber, foam, etc.), and part number or vendor.
11. What are the two **passive omni wheels** (front)? Same details.
12. What are the two **dead wheels** (passive odometry encoders)? Diameter, material, where they contact the ground, and how they are mounted to the frame.
13. How are the dead wheels mounted so they maintain consistent ground contact? Is there a spring-loaded or gravity-loaded mechanism?
14. How is the wheel track (center-to-center distance between the two driven wheels) measured? What is the exact measured value used in the firmware (the firmware uses 0.381 m — confirm or correct this)?
15. What is the gear ratio between the motor shaft and the drive wheels? How was this measured or calculated?

### 1.3 Sensor Mounting

16. How is the RPLIDAR A1M8 mounted? At what height from the ground (the URDF uses 527 mm — confirm or correct), forward/backward offset from the robot center (URDF uses +146.6 mm — confirm), and with what fasteners?
17. How is the XVF3800 USB microphone array mounted? Where on the robot body (front, top, facing direction), at what height, and with what fasteners?
18. How is the USB camera mounted? Location, height, facing direction, tilt angle, and fasteners.
19. How is the speaker mounted? Location, facing direction, and how it is secured.
20. Where is the STM32's IMU (BNO08x) physically located on the STM32 board or breakout? Is it mounted flat or at an angle? The URDF has it rotated 90° in roll — is this correct and why?

---

## Part 2: Electrical System & Power

### 2.1 Battery

21. What battery is used? Provide: chemistry (LiPo, Li-Ion, NiMH, etc.), nominal voltage, fully-charged voltage, cell count (e.g., 4S), capacity in mAh/Ah, and discharge rating (C rating).
22. Who is the manufacturer and what is the part number or product name?
23. What is the expected runtime per charge under normal operation?
24. How is the battery physically secured in the robot?
25. How is the battery connected to the power distribution system? (Connector type, wire gauge.)
26. Is there a Battery Management System (BMS) or protection circuit? If yes, describe it and provide the part number.
27. How is the battery charged? (Charger model, charging procedure, time to full charge.)
28. What is the safe minimum voltage before the battery must be charged?

### 2.2 Power Distribution

29. Draw or describe the power distribution topology: what voltage rails exist, what components are on each rail, and how power flows from the battery to each consumer.
30. Is there a main power switch? Where is it located on the robot body and what type is it (toggle, rocker, key switch)?
31. Are there any fuses or circuit breakers? If yes, what rating, where are they located, and what do they protect?
32. Are there any voltage regulators or DC-DC converters (e.g., battery is 4S ~16V and Jetson needs 5V or 9V)? For each converter: input voltage range, output voltage, maximum current, part number.
33. How is the Jetson Orin Nano powered? (USB-C PD, barrel jack, direct voltage — voltage level and connector type.)
34. How is the STM32 board powered? (From the Jetson USB, a separate regulator, the battery directly?)
35. How are the motors powered? (Directly from battery, through an ESC, through a motor driver board — voltage level.)
36. How is the RPLIDAR powered? (USB from Jetson, separate rail.)
37. How is the speaker powered? (From Jetson audio jack, separate amplifier board — if amplifier, provide part number.)

### 2.3 Motor System

38. What motors are used? Provide: type (brushed DC, brushless, stepper), manufacturer, model name, part number, nominal voltage, no-load RPM at nominal voltage, stall current, and shaft diameter.
39. How are the motors controlled? Provide the ESC or motor driver model/part number for each motor. Are the ESCs the same unit for both left and right?
40. How is the ESC connected to the STM32? (PWM signal wire, power, ground — which GPIO pins on the STM32, which connector pins on the ESC?) The firmware drives TIM14/CH1 for the left motor and TIM15/CH1 for the right — confirm this matches the wiring.
41. What is the full-forward and full-reverse pulse width for the ESC in microseconds? The firmware uses 1500 µs = stop, 1700 µs = forward max, 1300 µs = reverse max — confirm or correct.
42. Did the ESCs require any calibration (arming sequence, endpoint calibration)? If yes, describe the procedure.
43. Are the motor encoders (on the driven wheels, not the dead wheels) used for anything currently? The firmware reads them via TIM3 and TIM4 but odometry comes from the dead wheels. Confirm whether the motor encoders are active or ignored.

---

## Part 3: Electronics & PCB

### 3.1 STM32 Board

44. Is the STM32G0B1 on a custom PCB designed by the team, or a development board (e.g., Nucleo, Discovery)? If custom PCB, provide the schematic and PCB layout files (KiCad, Altium, Eagle, etc.) and where they are stored.
45. If a development board is used, provide the exact board name and purchase link.
46. List every connector on the STM32 board/PCB and what it connects to (motor ESCs, dead wheel encoders, IMU, battery ADC, USB to Jetson, power input, etc.).

### 3.2 Wiring

47. Provide a wiring diagram or wiring table showing every wire in the robot: from-connector, to-connector, wire color, wire gauge (AWG), and signal name.
48. How are wires managed inside the robot? (Cable ties, wire conduit, routed through frame channels.) Are there any known pinch points or wires that have been problematic?
49. What connectors are used for each subsystem? (JST, XT60, Dupont, Anderson Powerpole, etc.) List connector type and pin count for each connection.
50. What type of USB cable connects the Jetson to the STM32? (USB-A to USB-C, USB-C to USB-C, etc.) Is it a standard cable or does it need to be a specific length or quality?

### 3.3 Dead Wheel Encoder Wiring

51. What encoder is used on the dead wheels? Provide: manufacturer, part number, resolution (CPR before multiplexing), supply voltage, output type (quadrature, single-channel, open-collector, push-pull).
52. How are the dead wheel encoders connected to the STM32? List the exact wire connections: encoder pin → STM32 GPIO pin for each signal (A, B, VCC, GND) on both the straight and horizontal dead wheels. The firmware uses TIM1 (PA8/PA9) for the horizontal wheel and TIM2 (PC4/PA1) for the straight wheel — confirm.
53. Are pull-up or pull-down resistors needed on the encoder signals? Are they built into the STM32 GPIO configuration or on the PCB?

### 3.4 IMU

54. The firmware uses a BNO08x connected via I2C3 (SDA=PA6, SCL=PC0) at address 0x4B. Confirm this matches the hardware. Is this an Adafruit BNO085 breakout, a bare module, or something else?
55. What I2C address jumper/pin setting is used? (BNO08x has configurable address via PS0/PS1 pins.)

### 3.5 Battery Voltage Measurement

56. The firmware reads battery voltage through a resistor divider (100Ω + 22Ω) on STM32 pin PA4 (ADC1 channel 4). Confirm the resistor values and their placement (on the custom PCB, on a breadboard, or soldered inline). Provide the measured division ratio and the resulting ADC full-scale voltage for a fully charged battery.

---

## Part 4: Assembly & Maintenance

### 4.1 Assembly Order

57. Describe the correct order of assembly for the robot from a bare chassis. (What goes in first, what goes in last, what requires access to other components that must come before it.)
58. What tools are required for full assembly and disassembly? (Hex key sizes, screwdriver types, soldering iron, etc.)
59. How long does full assembly take from parts to a running robot?

### 4.2 Critical Fastening Notes

60. Which fasteners or adhesives require special attention? The dead wheels and drive wheels were previously secured with Loctite to prevent drift — document exactly where Loctite is applied and what grade (e.g., Loctite 243 medium strength).
61. Are any parts press-fit, glued, or interference-fit? If yes, describe how to disassemble them safely.

### 4.3 Removing and Reinstalling the Jetson

62. Step-by-step: how does a technician remove the Jetson from the robot for WiFi reconfiguration or maintenance? What needs to be disconnected, in what order, and what is the risk of damaging connectors?
63. Step-by-step: how is the Jetson reinstalled and verified to be properly seated?

### 4.4 Replacing Components

64. If a dead wheel is replaced, what calibration is needed afterward? (The firmware uses `DW_WHEEL_DIAMETER = 0.0814 m` as a calibrated value — document how this was measured.)
65. If a motor or ESC is replaced, what calibration or firmware change is needed?
66. If the battery is replaced, what must be checked or updated? (Voltage divider ratio, software thresholds.)

---

## Part 5: Safety & Handling

67. What are the electrical safety precautions when working on SAGE with the battery connected? Are there any capacitors that remain charged after power-off?
68. What is the safe handling procedure for the battery? (Storage voltage, temperature limits, puncture risk.)
69. Are there any pinch points or crush hazards when the motors are running? Where?
70. What is the maximum speed SAGE can reach, and what is the software-limited maximum speed used in normal operation?
71. Are there any mechanical limits or hard stops that prevent the robot from being driven into a wall at full speed?

---

## Part 6: Photographs & Diagrams

Please provide the following visual assets with this document:

- [ ] Top-down photo of the fully assembled robot (no lid)
- [ ] Photo with the lid on, as deployed
- [ ] Photo of the electronics bay showing all major components and their locations
- [ ] Photo of each dead wheel assembly showing the mounting mechanism
- [ ] Photo of the motor and ESC wiring
- [ ] Full wiring diagram (hand-drawn is acceptable, but labeled clearly)
- [ ] Power distribution diagram
- [ ] 3D CAD render or exploded view showing all major structural parts
- [ ] Photo of the STM32 board / custom PCB (top and bottom if custom)
- [ ] Photo of the battery and its compartment

---

*Fill in this questionnaire completely and attach all files before submitting. This document becomes part of the permanent SAGE project record and will be referenced in all future maintenance, repair, and handoff situations.*
