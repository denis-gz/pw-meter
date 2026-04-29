# PW Meter - EMB_1 Graduation Project

![Splash Screen](/resources/dither_it_Gemini_Generated_Image_hjzm7lhjzm7lhjzm.png)

## Power Meter - An appliance to measure and log AC power line parameters

The Power Meter can be attached to the AC mains load to measure the following parameters:

 - **AC Voltage** (V, RMS)
 - **AC Current** (A, RMS)
 - **Instantaneous active power** (W)
 - **Instantaneous apparent power** (VA)
 - **Accumulated energy** (Wh)
 - **Power factor** (W/VA ratio)
 - **AC frequency** (Hz)

## Quick Start

In case you have assembled the board using the schematic and components specified in the KiCad project, follow these steps to bring it to life:

1. You may use the supplied firmware image or build your own. If building from source, please update `mqtt_creds.h` and `wifi_creds.h` files with your credentials (not necessary if you do not require MQTT telemetry). Also, please verify settings in the `sdkconfig` to match actual hardware (since at the time of this writing the project has not yet been tested with the actual/final PCB). Use usual `idf.py menuconfig` command to re-generate the `sdkconfig`. See also [defconfig.bak](defconfig.bak) file for exact set of parameters used to configure the project.
2. To build the firmware image, use standard **ESP-IDF** tools (CMake, idf.py, etc). Look [here](https://docs.espressif.com/projects/esp-idf/en/v5.5.4/esp32/api-guides/build-system.html) for a reference.
3. Flash the firmware via USB Serial/JTAG interface (hold the **BOOT** key while powering on the board). The ESP32's **UART0** interface is also available on the **J3** header for flashing or serial console access (useful for adjusting parameters at runtime).
4. Connect the AC voltage source to the Voltage Transformer module (**U6** sub-board).
5. Pass the **Live** wire of the AC load through the core of the Current Transformer (**SCT013-100** or similar). For low-power loads, wrap the wire around the transformer core several times to amplify the signal. Alternatively, use a sensor with a lower current range. If your sensor includes an internal burden resistor, you may need to desolder the **R3** resistor from the board. Connect the Current Transformer to the **J2** socket. It is also possible to use the PW Meter without a load; in this case, set the `I noise floor` parameter high enough to prevent bogus readings.
6. Connect a **7-30V DC** power source to the **J1** socket. The board itself should not consume more than 5 Watts.
7. The Power Meter starts working immediately, displaying data on the OLED display.
8. Use the **Encoder** to switch pages. Use the Encoder's **push-button** to pause/resume on the Mains page, or to enter the menu on the Settings page. Use a **long press** (> 0.5 sec) to return to the upper level. Performing a long press while at the top level will restart the app. Use the **RES** key for a hard reset.

## Project Structure

### Hardware

The project consists of a main board and two sub-boards:
* **Voltage Transformer:** Single-phase AC Active Output Voltage Transformer by [LC Technology](http://www.chinalctech.com/cpzx/Programmer/Sensor_Module/250.html).
* **Display:** SSD1315 128x64 monochrome I2C OLED (GM009605). Any other SSD1306 I2C compatible display should also work (check the module's address in `idf.py menuconfig`).

Sampling of AC voltage and current channels occur in DMA continuous mode, the rate is 4 kHz per channel, with 12 bit resolution and 11.5 dB attenuation. The samples are then adjusted for non-linearity using pre-calibrated curves, centered at 0 level, and processed using DSP capabilities of ESP32-S3 chip. Employing hardware acceleration helps keep CPU usage reasonably low (~10% on Core 1, or only 5% without display output).

Please find further details on project's [Schematic](docs/hardware/Schematic_R1.pdf), and PCB board views ([Top Layer](docs/hardware/PcbTop.png), [Inner 1](docs/hardware/PcbInner1.png), [Inner 2](docs/hardware/PcbInner2.png), [Bottom Layer](docs/hardware/PcbBottom.png) and [3D View](docs/hardware/jlcpcb_3d_1.png)).

### Software

The software consists of 5 tasks:

* `reader_task`: ADC sampling task. Performs reading, parsing and adjustment of analog data. Outputs raw values in mV to a ring buffer for the Compute Task.
* `compute_task`: Processes data in 10-cycle chunks. Performs all mathematical calculations and outputs results to the queue for the Interface Task.
* `interface_task`: Applies filtering, updates the OLED display, handles user input from the rotary encoder and console. Sends a copy of the UI data to the Telemetry Task.
* `telemetry_task`: Maintains Wi-Fi and MQTT connections and publishes data to the MQTT broker.
* `console_input_task`: A service task created by the console event loop that handles prompts and waits for user input from the UART console.

The `PowerMeterApp` class represents the main class which initializes all tasks and waits for a stop event. It handles task shutdown, and the `app_main()` function recreates the instance if needed.

## Signal Path

![Signal Path](/docs/SignalPathDiagram.png)

## Data Flow

![Data Flow](/docs/DataFlowDiagram.png)

## Technology

* **MCU:** Espressif ESP32-S3-WROOM-1 N16R2 module
* **AC Voltage Input:** ZMPT101B transformer paired with an LM358 op-amp
* **AC Current Input:** [SCT013-100](http://en.yhdc.com/comp/file/download.do?id=941) Split-core current transformer (100A, 2000:1 ratio)
* **Display:** GM009605 OLED module based on SSD1315 controller (I2C bus)
* **Input:** PEC11R rotary encoder
* **Debugging:** BOOT and RESET buttons, UART, and USB Type-C JTAG/Serial ports
* **Software:** ESP-IDF v5.5.4-based CMake project, developed with QtCreator v18.0.2
* **Third-party components:**
    * `nopnop2002/ssd1306`: OLED display support
* **PCB:** 4-layer FR-4 PCB (100x50 mm), developed in KiCad 9.0.7. Mostly uses SMT components (0805 and 0603).

## Example Output

![Splash Screen](/docs/photos/IMG_20260428_061512.jpg) ![Mains Page](/docs/photos/IMG_20260428_042619_479.jpg) ![Device Page](/docs/photos/IMG_20260428_042630_639.jpg) ![Network Page](/docs/photos/IMG_20260428_042640_663.jpg) ![Settings Page](/docs/photos/IMG_20260428_061856_930.jpg) ![Parameter Page](/docs/photos/IMG_20260428_061908_271.jpg) ![Log Page](/docs/photos/IMG_20260428_042702_820.jpg) ![About Page](/docs/photos/IMG_20260428_042713_516.jpg)

## Author

Copyright © Denys Zavorotnyi, 2026  

## License

[MIT License](LICENSE)
