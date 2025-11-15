# gesture-robot
Hand-gesture-controlled robot: ESP32 (TCP server) + PC MediaPipe client (Python). Uploads motor commands to ESP32 over WiFi.
# Gesture Robot (ESP32 + MediaPipe client)

This project contains code to control a robot using hand gestures:

- **backend-esp32/** — ESP32 Arduino sketch (TCP server). Receives `left|right\n` commands and drives motors.
- **client-pc/** — MediaPipe Python client. Detects hand direction via webcam and sends `left|right\n` to ESP32.
- **docs/** — wiring diagrams and screenshots.

## Quick start (beginner-friendly)

### 1) Upload ESP32 sketch
1. Open `backend-esp32/GestureBackend.ino` in Arduino IDE.
2. Select your ESP32 board and COM port.
3. Upload.

ESP32 will create a WiFi AP named `ESP32_AP` (password `12345678`) and run a TCP server on port **81**.

### 2) Run the PC client
On your PC:
```bash
cd client-pc
python -m venv venv
# macOS / Linux:
source venv/bin/activate
# Windows (PowerShell):
venv\Scripts\Activate.ps1

pip install -r requirements.txt
python gesture_client.py
```

Look at the OpenCV window; use hand gestures (UP/LEFT/RIGHT/DOWN) to move the robot.

## Files
- `backend-esp32/GestureBackend.ino` — ESP32 sketch
- `client-pc/gesture_client.py` — MediaPipe client (webcam)
- `client-pc/requirements.txt` — Python deps
- `docs/` — wiring images and notes

## Test without webcam
Use `client-pc/tools/test_sender.py` (example included) to send commands to the ESP without camera.

## License
MIT
