# HUNO ESP32-S3 Controller

This project is an Arduino C++ port of the HUNO robot controller, designed to run on the ESP32-S3 microcontroller. It provides low-level servo communication, a walking engine (Parallel Kinematics), motion recording/playback, and joint calibration.

## Project Structure

- **`esp32s3.ino`**: The main Arduino sketch. It ties all components together and provides a Serial monitor interface.
- **`WCK.h` & `WCK.cpp`**: Implements the WCK servo communication protocol over Hardware UART.
- **`PK.h` & `PK.cpp`**: The Parallel Kinematics walk engine. It calculates joint angles for walking and updates them asynchronously.
- **`Calib.h` & `Calib.cpp`**: Calibration tool for the robot. It allows interactive setting of joint zero-positions and saves them persistently to the ESP32's flash using the `Preferences` library.
- **`Motion.h` & `Motion.cpp`**: Motion playback and generation. It can parse PLEN2 JSON motion formats directly from `LittleFS` using `ArduinoJson`, and can print current poses to aid in generating new motions.
- **`Config.h` & `Config.cpp`**: Global configurations, enumerations for joint names, and default zero offsets.

## Hardware Setup

1. **Microcontroller**: ESP32-S3
2. **Servo Connection**: The WCK servos require a half-duplex UART connection. Ensure your circuit handles the TX/RX line properly if the servos require a single data line, or wire them appropriately to the defined TX/RX pins.
3. **Pin Configuration**: Open `esp32s3.ino` and configure the UART pins to match your physical wiring:
   ```cpp
   #define WCK_RX 16
   #define WCK_TX 17
   ```

## Software Dependencies

Before compiling in the Arduino IDE, you must install the **ArduinoJson** library:
1. Open Arduino IDE.
2. Go to **Sketch** -> **Include Library** -> **Manage Libraries...**
3. Search for **ArduinoJson** (by Benoit Blanchon).
4. Click **Install** (Version 6.x or newer is recommended).

## Building & Flashing

1. Open `esp32s3/esp32s3.ino` in the Arduino IDE.
2. Select your exact ESP32-S3 board from **Tools** -> **Board**.
3. (Optional) If you plan to load JSON motion files onto the device, ensure you select a partition scheme that includes LittleFS/SPIFFS (e.g., `Default 4MB with spiffs`). Use a filesystem uploader plugin to upload your `.json` files to the data partition.
4. Click **Upload**.

## Usage & Commands

Open the Arduino IDE **Serial Monitor**. Set the baud rate to **115200** and ensure the line ending is set to **Newline** or **Both NL & CR**. 

You can interact with the robot by sending the following commands:

- `w` : Start the walking sequence.
- `s` : Stop the walking sequence.
- `z` : Go to the zero/ready position.
- `c` : Enter the interactive calibration menu.
    - Inside the menu, send a joint ID (`0` to `15`) to select it.
    - Send `w` to increase the position and `z` to decrease the position.
    - Send `s` to save the new offsets to flash memory.
    - Send `x` to exit calibration.
- `p` : Passivate motors. This turns off torque so you can manually move the robot to generate keyframes.
- `r` : Read current pose. Prints the current servo positions in a PLEN2-compatible JSON array format, which you can copy-paste to create motion files.
- `m <filepath>` : Play a motion file (e.g., `m /motion.json`). This requires the file to be present on the ESP32's LittleFS filesystem.
