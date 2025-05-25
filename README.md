# Smart Street Lights System

This repository contains the hardware firmware and Flask-based web application for the Smart Street Lights System, an IoT project designed to optimize street lighting by automatically adjusting brightness based on ambient conditions and pedestrian traffic. Additionally, it includes emergency alert functionalities.

## Repository Structure

* `with_analysis.ino`: Arduino firmware code for ESP32-S3 microcontroller.
* `app.py`: Flask backend application.
* `templates/`: Directory containing HTML template files for the Flask app.
* `requirements.txt`: Python dependencies for Flask server.

## Hardware Requirements

* **Microcontroller:** XIAO ESP32-S3 Sense
* **Sensors:**

  * Ambient Light Sensor (PiicoDev VEML6030)
  * Distance Sensor (PiicoDev VL53L1X)
* **Actuators:** White LED (5mm)
* **Miscellaneous:** Push-button, jumper wires, breadboard

## Installation Instructions

### Hardware Setup

1. Connect sensors via I2C (SDA, SCL) to ESP32-S3.
2. Connect LED with resistor (220Ω) to a GPIO pin.
3. Connect push-button to a GPIO pin with internal pull-up enabled.
4. Power ESP32 via USB.

### Firmware Installation

1. Install Arduino IDE and ESP32 support.
2. Open `with_analysis.ino` in Arduino IDE.
3. Install required libraries (`VEML6030`, `VL53L1X`, `ESP-NOW`, `Wi-Fi`).
4. Set Wi-Fi credentials and Flask server IP in the code.
5. Upload to ESP32.

### Flask Server Setup

1. Ensure Python 3.x is installed.
2. Clone or download this repository.
3. Navigate to the project directory:

```sh
cd flask_iot_server
```

4. Install dependencies:

```sh
pip install -r requirements.txt
```

5. Run the Flask server:

```sh
python app.py
```

6. Access the dashboard at:

```
http://localhost:5000
```

## Usage

* Monitor street lights status in real-time.
* Manually control lights via the dashboard.
* Receive emergency alerts triggered from the hardware.

## Contributions

Contributions are welcome. Please create an issue or submit a pull request.

## License

This project is available under the MIT License.
