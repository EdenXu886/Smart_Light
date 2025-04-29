# Smart Light Complete Project

This repository includes:

- **motion_sensor/**: ESP32 firmware to detect motion and control lights via WiFi.
- **smartlight-dashboard/**: React web app to monitor and control smart lights.

---

## Running the Project

### 1. ESP32 Firmware (motion_sensor)

- Open `motion_sensor/motion_sensor.ino` in Arduino IDE
- Install required libraries: `WiFi.h`, `WebServer.h`, `Wire.h`, `Adafruit Sensor`, `BH1750`
- Set your WiFi credentials in the code
- Upload to your ESP32 board

### 2. React Dashboard (smartlight-dashboard)

- Go into `smartlight-dashboard/` directory
- Install dependencies:

```bash
npm install
```

- Start development server:

```bash
npm run dev
```

- Open the browser at `http://localhost:5173`

- Make sure your React app uses the correct ESP32 IP address.

---

Enjoy your smart lighting system! 🚀
