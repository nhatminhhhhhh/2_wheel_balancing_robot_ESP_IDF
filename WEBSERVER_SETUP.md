# PID Web Server Setup Instructions

## 1. Update WiFi Credentials

Open `main/webserver.h` and update these lines with your WiFi router credentials:

```c
#define WIFI_SSID      "YOUR_WIFI_SSID"      // Change to your WiFi name
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"  // Change to your WiFi password
```

## 2. Build and Flash

Build and flash the project as usual:
```bash
idf.py build
idf.py flash monitor
```

## 3. Find ESP32 IP Address

After flashing, check the serial monitor for the IP address. You'll see:
```
I (xxx) WEBSERVER: Got IP:192.168.1.xxx
I (xxx) MAIN: WiFi connected! Starting web server...
I (xxx) WEBSERVER: Web server started successfully!
I (xxx) MAIN: Web server started. Access PID tuning at http://[ESP32_IP]
```

## 4. Access Web Interface

Open a web browser on any device connected to the same WiFi network and navigate to:
```
http://192.168.1.xxx
```
(Replace with the IP shown in your serial monitor)

## 5. Tune PID Parameters

The web interface provides:
- **Kp slider**: 0 to 100, step 0.1
- **Ki slider**: 0 to 100, step 0.1
- **Kd slider**: 0 to 10, step 0.001

You can either:
- Use the sliders to adjust values
- Type exact numbers in the input boxes
- Click "Apply PID Parameters" to send values to ESP32

## 6. Real-time Updates

PID parameters are updated in real-time. The ESP32 will use the new values immediately in the PID control loop.

## Features

✅ Mobile-friendly responsive design
✅ Real-time parameter adjustment
✅ Both slider and numeric input
✅ Visual feedback on parameter updates
✅ No need to recompile/reflash code
