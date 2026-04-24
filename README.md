# PW Meter - EMB_1 graduation project

Power Meter - an appliance to measure and log the mains AC grid parameters

## The appliance can be attached to the AC mains load, and allows to measure the following parameters:

 - Voltage (V)
 - Current (A)
 - Instantious active power (W)
 - Instantious apparent power (VA)
 - Power factor, both true and displacement (W/VA and cos_φ)
 - AC frequency (Hz)
 - Total Harmonic Distortions (%)


## Quick Start

1. Connect AC voltage to the Voltage transformer module (U6)
2. Choose a Line wire of AC load and put it inside the core of current transformer. Connect the current transformer to J2 socket.
3. Connect 7-30V DC power to the J1 socket
4. The Power Meter starts working immediately, displaying the data on OLED display

## Architecture

The project consists of main board (CPU, OLED, Rotary encoder) and a sub-board - the Single-phase AC Active Output Voltage Transformer Module made by LC Technology [http://www.chinalctech.com/cpzx/Programmer/Sensor_Module/250.html]. For current input, the Split-core current transformer of type SCT013-100 is necessary [http://en.yhdc.com/comp/file/download.do?id=941]

The software part consist of 5 tasks:
 - ADC sampling task (performs reading, parsing and curve-adjusting analog data, outputs raw values in mV to the ring buffer, to be consumed by Compute Task)
 - Computation task (processes data by 10-cycles chunks, performs all the math, outputs results to the queue, to be consumed by the Display Task)
 - Display task (applies statistical EMA filter, presents data on OLED display, prints to UART console)
 - Telemetry task (publishes data to the MQTT broker)
 - UI task (accepts user input from rotary encoder, presents Settings Menu or Main screen)

## Docs

TBDL

## Technology

 - CPU: Espressif ESP32-S3-WROOM-1 N16R2 module
 - AC voltage input: ZMPT101B transformer, paired with LM358 op-amp
 - AC current input: Split-core current transformer of type SCT013-100 (100A, 2000:1 ratio)
 - Display: GM009605 OLED module, connected by I2C bus
 - Input: PEC11R rotary encoder
 - Debugging and firmware update: BOOT and RESET buttons, UART and USB Type-C JTAG/Serial ports
 - Software: ESP-IDF v5.5.4-based CMake project, developed with QtCreator v18.0.2
 - Hardware: 4-layer standard type FR-4 PCB, sized 100x50 mm, developed in KiCad 9.0.7, fabricated and assembled using JLCPCB services. Mostly SMT components used, of size 0805 and 0603.
 - Third-party components:
   - nopnop2002/ssd1306 - OLED display support
   - esp-idf-lib/i2cdev - I2C bus support

## Results

TBDL

## Author

Copyright © Denys Zavorotnyi, 2026

E-mail: denis.gz@gmail.com

## License

TBDL
