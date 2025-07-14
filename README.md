# RC Car Bluetooth Control (ESP32)

This project provides the Arduino code for controlling an RC car via Bluetooth using an ESP32 microcontroller. The car's movement and speed can be controlled wirelessly from a Bluetooth-enabled Android device using the "Braulio Calle Bluetooth RC Controller" (or similar generic Bluetooth serial terminal) application.

---
## Features

* **Bluetooth Connectivity:** Establishes a Bluetooth Serial connection for wireless control.
* **Speed Control:** Allows for granular control over the car's speed through specific Bluetooth commands, mapped to a slider or custom buttons in the control app.
* **Directional Control:** Supports various movements including forward, backward, left, right, and diagonal movements (forward-right, backward-right, forward-left, backward-left), mapped to custom buttons in the control app.
* **Modular Design:** Clearly defined functions for each movement make the code easy to understand and modify.
* **PWM Motor Control:** Utilizes the ESP32's LEDC peripheral for precise Pulse Width Modulation (PWM) control of the motors, ensuring smooth speed adjustments.

---
## Hardware Requirements

* **ESP32 Development Board:** The core of the project, responsible for Bluetooth communication and motor control.
* **Motor Driver Module:** An H-bridge motor driver (e.g., L298N, DRV8833, or similar) is required to control the DC motors. The ESP32's pins cannot directly supply enough current for the motors.
* **DC Motors (x2 or x4):** Depending on your car's chassis design (two-wheel drive or four-wheel drive). This code assumes a two-wheel differential drive setup with two motors, each controlled by two PWM pins (one for forward, one for reverse).
* **RC Car Chassis:** The physical platform for your robot.
* **Power Supply:** A suitable battery pack for the ESP32 and the motors. Ensure the motor driver and motors are powered correctly, usually separately from the ESP32 but with common ground.
* **Jumper Wires:** For connecting components.

---
## Software Requirements

* **Arduino IDE:** The development environment for writing and uploading the code to the ESP32.
* **ESP32 Boards Manager:** You'll need to install the ESP32 board definitions in the Arduino IDE. Go to `File > Preferences > Additional Boards Manager URLs` and add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`. Then go to `Tools > Board > Boards Manager` and search for "esp32" to install it.
* **BluetoothSerial Library:** This library is typically included with the ESP32 core.
* **Android App:** "Braulio Calle Bluetooth RC Controller" (or "Arduino Bluetooth Controller") APK or from the Google Play Store.

---
## Pinout

The following pins are used for motor control. You may need to adjust these based on your specific ESP32 board and motor driver connections.

* `R1PWM` (Right Motor Forward PWM): GPIO **19**
* `R2PWM` (Right Motor Backward PWM): GPIO **21**
* `L1PWM` (Left Motor Forward PWM): GPIO **23**
* `L2PWM` (Left Motor Backward PWM): GPIO **22**

---
## Getting Started

### 1. Wiring the Components

Connect your ESP32, motor driver, and motors according to the following general scheme. **Always double-check your motor driver's datasheet for specific wiring instructions.**

* **ESP32 to Motor Driver:**
    * Connect ESP32 GPIO 19 to your motor driver's input for Right Motor Forward.
    * Connect ESP32 GPIO 21 to your motor driver's input for Right Motor Backward.
    * Connect ESP32 GPIO 23 to your motor driver's input for Left Motor Forward.
    * Connect ESP32 GPIO 22 to your motor driver's input for Left Motor Backward.
    * Connect ESP32 GND to Motor Driver GND.
* **Motor Driver to Motors:**
    * Connect the motor driver's output for the right motor to your right DC motor.
    * Connect the motor driver's output for the left motor to your left DC motor.
* **Power Connections:**
    * Connect your motor power supply to the motor driver's power input (e.g., `VCC` or `VM`).
    * Power your ESP32 (e.g., via USB or a separate 3.3V/5V power supply, depending on your board).
    * Ensure all grounds are common.

### 2. Uploading the Code

1.  Open the provided `.ino` file in the Arduino IDE.
2.  Select your ESP32 board from `Tools > Board > ESP32 Arduino`. For example, "ESP32 Dev Module".
3.  Select the correct COM port from `Tools > Port`.
4.  Click the "Upload" button (right arrow icon) to compile and upload the code to your ESP32.

### 3. Connecting via Bluetooth using the App

Once the code is uploaded and your ESP32 is powered on, it will broadcast itself as a Bluetooth device named **"My Soccer Bot YT"**.

To control the car:

1.  **Install the "Braulio Calle Bluetooth RC Controller" App:** Download and install this application from the Google Play Store on your Android smartphone.
2.  **Pair with the ESP32:** Open your phone's Bluetooth settings and pair with "My Soccer Bot YT". The default pairing code is usually `1234` or `0000`, but for ESP32 with `BluetoothSerial`, no pairing code might be required directly from the phone's Bluetooth settings for initial connection within the app.
3.  **Connect in the App:**
    * Open the "Braulio Calle Bluetooth RC Controller" app.
    * Navigate to the connection screen (usually by tapping a "Connect" or Bluetooth icon).
    * Select "My Soccer Bot YT" from the list of paired or available devices.
    * The app should indicate a successful connection.
4.  **Configure the App's Controls:**
    * This app often has customizable buttons and sliders. You will need to map the buttons to send the specific characters that your ESP32 code expects.
    * **For Movement:** Configure buttons to send single characters like `F`, `B`, `L`, `R`, `S`, `I`, `J`, `G`, `H`. The app typically allows you to set the character sent when a button is pressed.
    * **For Speed Control:** You can use a slider control within the app if available, or dedicate specific buttons to send the speed characters (`0` through `q`). If using a slider, you might need to map its range to your desired speed characters or adjust your Arduino code to handle a numerical value from the slider directly. For this code, individual buttons for each speed increment are expected.
    * **Example (Button Configuration):** In the app's settings for a button, you would set:
        * Button for Forward: Sends `F`
        * Button for Backward: Sends `B`
        * Button for Stop: Sends `S`
        * ...and so on for all movement commands.
        * Button for Speed 100: Sends `0`
        * Button for Speed 255: Sends `q`

---
## Control Commands

Once connected via the Bluetooth serial terminal or configured app buttons, send the following single-character commands to control your RC car:

### Movement Commands:

| Command | Action             | Description                                    |
| :------ | :----------------- | :--------------------------------------------- |
| `F`     | **Go Forward** | Moves the car straight forward.                |
| `B`     | **Go Backward** | Moves the car straight backward.               |
| `L`     | **Go Left** | Pivots the car to the left.                    |
| `R`     | **Go Right** | Pivots the car to the right.                   |
| `S`     | **Stop** | Halts all motor movement.                      |
| `I`     | **Forward Right** | Only the right motor moves forward.            |
| `J`     | **Backward Right** | Only the right motor moves backward.           |
| `G`     | **Forward Left** | Only the left motor moves forward.             |
| `H`     | **Backward Left** | Only the left motor moves backward.            |

### Speed Control Commands:

These commands set the global `Speed` variable, which determines the PWM duty cycle for motor control. The `Speed` value ranges from 0 (stopped) to 255 (full speed).

| Command | Speed Value |
| :------ | :---------- |
| `0`     | 100         |
| `1`     | 110         |
| `2`     | 120         |
| `3`     | 130         |
| `4`     | 140         |
| `5`     | 150         |
| `6`     | 180         |
| `7`     | 200         |
| `8`     | 220         |
| `9`     | 240         |
| `q`     | 255 (Max Speed)|

---
## Code Explanation

### Global Variables

* `BluetoothSerial serialBT;`: An instance of the `BluetoothSerial` library for handling Bluetooth communication.
* `char BT;`: Stores the incoming character received via Bluetooth.
* `int Speed = 100;`: Initializes the motor speed. This value can be changed by speed control commands.
* `int R1PWM = 19;`, `int R2PWM = 21;`, `int L1PWM = 23;`, `int L2PWM = 22;`: These define the GPIO pins connected to the motor driver for controlling the right and left motors (forward and backward).

### `setup()` Function

* `Serial.begin(115200);`: Initializes serial communication for debugging purposes (you can view output in the Arduino IDE's Serial Monitor).
* `serialBT.begin("My Soccer Bot YT");`: Starts the Bluetooth serial service and sets the visible name of your ESP32 device.
* `pinMode(..., OUTPUT);`: Configures the specified motor control pins as outputs.
* `ledcAttach(pin, frequency, resolution);`: This is crucial for ESP32 PWM control.
    * `R1PWM`, `R2PWM`, `L1PWM`, `L2PWM`: The GPIO pins to attach to a PWM channel.
    * `5000`: The PWM frequency (5 kHz).
    * `8`: The PWM resolution (8 bits), meaning the duty cycle can range from 0 to $2^8 - 1 = 255$.
* Error handling for `ledcAttach`: If any `ledcAttach` fails, it prints an error and halts the program.

### `loop()` Function

* `while (serialBT.available())`: Continuously checks if there is data available to read from the Bluetooth serial buffer.
* `BT = serialBT.read();`: Reads the incoming character from the Bluetooth buffer.
* **Speed Control Logic:** A series of `if` statements check the received character (`BT`) and update the `Speed` variable accordingly.
* **Movement Control Logic:** An `if-else if` ladder checks the received character and calls the appropriate motor control function (`go_forward()`, `stop()`, etc.).

### Motor Control Functions (`go_forward()`, `go_backward()`, etc.)

These functions manipulate the motor speed by writing PWM duty cycle values to the specific motor pins using `ledcWrite(pin, value);`.

* `ledcWrite(pin, Speed);`: Sets the PWM duty cycle for the specified pin to the current `Speed` value, driving the motor.
* `ledcWrite(pin, 0);`: Sets the PWM duty cycle to 0, effectively stopping that specific motor or direction.

---
## Troubleshooting

* **No Bluetooth Connection:**
    * Ensure the ESP32 is powered on.
    * Verify the Bluetooth name "My Soccer Bot YT" is correct in `serialBT.begin()`.
    * Make sure you've installed the ESP32 board definitions in Arduino IDE.
    * Try restarting the ESP32 and your phone's Bluetooth.
    * Ensure you've correctly paired the device in your phone's Bluetooth settings **before** connecting within the app.
* **Motors Not Moving:**
    * **Wiring:** Double-check all wiring connections, especially between the ESP32, motor driver, and motors. Ensure common grounds.
    * **Power Supply:** Verify that both the ESP32 and the motor driver/motors have adequate power supplies. Motors typically require a higher voltage and current than the ESP32 can provide.
    * **Motor Driver:** Is the motor driver powered correctly? Is it enabled (some drivers have an enable pin)?
    * **Code:** Check the `ledcAttach` and `ledcWrite` calls for correct pin numbers.
    * **`ledcAttach` Failure:** If you see "LEDC setup failed!" in the Serial Monitor, there's an issue with setting up PWM channels. This could be due to incorrect pin numbers or a corrupted ESP32 core installation.
* **Motors Move in Wrong Direction:**
    * Swap the two wires connecting to the problematic motor on your motor driver.
    * Alternatively, you can reverse the logic in the code (e.g., swap `R1PWM` and `R2PWM` in the `go_forward()` function for the right motor).
* **Unexpected Behavior / Glitches:**
    * **Bluetooth interference:** Other Bluetooth devices nearby might cause issues.
    * **Power fluctuations:** Unstable power can lead to unpredictable behavior.
    * **Software issues:** Ensure you have the latest ESP32 Arduino core.
    * **App Configuration:** Double-check that the commands sent by the app's buttons/sliders exactly match the characters expected by your Arduino code (e.g., `F` for forward, not `f`).

---
## Further Enhancements

* **PID Control:** Implement PID (Proportional-Integral-Derivative) control for more precise speed and direction control, especially for maintaining straight lines or specific turning radii.
* **Accelerometer/Gyroscope:** Add an IMU (e.g., MPU6050) for self-balancing capabilities or more advanced navigation.
* **Obstacle Avoidance:** Integrate ultrasonic sensors (HC-SR04) or IR sensors to enable autonomous obstacle avoidance.
* **Battery Monitoring:** Add voltage dividers to monitor the battery level and provide warnings.
* **Web-based Control:** Explore using ESP-NOW or Wi-Fi for control if you need longer range or more complex data exchange.
* **LED Indicators:** Add LEDs to indicate connection status, current speed mode, or motor activity.

---
## License

This project is open source and available under the MIT License. Feel to use, modify, and distribute it for your own projects.

---
