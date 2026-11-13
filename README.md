# ESP32 GC2145 Camera Server

PlatformIO firmware and desktop tooling for an ESP32-CAM with a GC2145 camera
sensor. The repository documents two viewing paths:

1. **Web-hosted stream:** the ESP32 hosts a browser-viewable grayscale MJPEG
   stream over HTTP.
2. **UDP viewer:** the Python utility receives camera frames over UDP and hosts
   a local browser viewer (and optional OpenCV windows).

## Features

- GC2145 camera configuration for the `esp32cam` PlatformIO board
- HVGA capture when PSRAM is available
- JPEG conversion and multipart MJPEG streaming
- Static IP configuration
- Browser page at `/` and stream endpoint at `/stream`
- Serial diagnostics for Wi-Fi signal, stream FPS, heap, and PSRAM
- UDP frame receiver with a local web viewer on port `5000`

## Hardware

The firmware uses the following GC2145 pin mapping:

| Signal | GPIO |
| --- | ---: |
| PWDN | 32 |
| XCLK | 0 |
| SCCB SDA | 26 |
| SCCB SCL | 27 |
| D0-D7 | 5, 18, 19, 21, 36, 39, 34, 35 |
| VSYNC | 25 |
| HREF | 23 |
| PCLK | 22 |

The camera is configured for grayscale capture, image mirroring, and vertical
flip.

## Requirements

- ESP32-CAM hardware with a GC2145 sensor
- VS Code with the PlatformIO IDE extension
- USB-to-serial adapter or compatible upload connection
- A 2.4 GHz Wi-Fi network

## Configuration

Before flashing, update the Wi-Fi and network settings near the top of
[`src/main.cpp`](src/main.cpp):

```cpp
const char* WIFI_SSID = "your-network";
const char* WIFI_PASS = "your-password";

#define USE_STATIC_IP true
IPAddress local_IP(192, 168, 0, 101);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
```

Choose an unused address on the same subnet as the router. Set
`USE_STATIC_IP` to `false` to use DHCP.

Do not commit real Wi-Fi credentials to source control.

## Build, upload, and monitor

The project is configured in [`platformio.ini`](platformio.ini) for the
`esp32cam` environment:

```bash
pio run
pio run --target upload
pio device monitor
```

The serial monitor uses `115200` baud. The configured upload port is
`/dev/ttyUSB0`; change `upload_port` in `platformio.ini` if the device appears
on a different port.

## Web-hosted camera stream

1. Power the board and open the serial monitor.
2. Wait for `Camera server ready!` and the printed IP address.
3. Open `http://<device-ip>/` in a browser.
4. Use `http://<device-ip>/stream` for the raw MJPEG endpoint.

The firmware restarts if it cannot connect to Wi-Fi within 20 seconds. During
operation, the serial monitor reports stream FPS, RSSI, free heap, and free
PSRAM once per second.

## UDP camera viewer

[`script/udp_viewerfps.py`](script/udp_viewerfps.py) is a second, separate
viewing path. It listens for a UDP strip-framing protocol on port `2222`,
reassembles grayscale frames, and serves them as MJPEG from a local web server
on port `5000`. It can also display OpenCV desktop windows.

The UDP viewer and ESP32 web server are separate protocols. The current
firmware in [`src/main.cpp`](src/main.cpp) sends HTTP MJPEG and does not send
the UDP packets expected by this script. Use the UDP viewer with firmware or
another sender that implements the script's packet format.

Install the viewer dependencies and run:

```bash
python3 -m pip install flask opencv-python numpy
python3 script/udp_viewerfps.py
```

Then open `http://localhost:5000/` for the locally hosted viewer. The UDP
sender must be reachable on the same network and send to the computer running
the script on UDP port `2222`.

## Choosing a viewing path

| Path | Sender | Viewer URL | Protocol |
| --- | --- | --- | --- |
| ESP32 web server | Current `src/main.cpp` firmware | `http://<device-ip>/` | HTTP MJPEG |
| Desktop UDP viewer | Compatible UDP camera firmware/sender | `http://localhost:5000/` | UDP input, local HTTP output |

Both paths can be used on the same network, but the current firmware must be
extended with a UDP sender before it can feed the Python viewer.

## Project layout

```text
src/main.cpp            ESP32 camera, Wi-Fi, and HTTP server firmware
script/udp_viewerfps.py Optional UDP receiver and desktop/web viewer
platformio.ini          PlatformIO build and upload configuration
```

## Troubleshooting

- **Camera initialization failed:** check the camera model, wiring, and
  selected board configuration.
- **No Wi-Fi connection:** verify credentials and static network settings.
- **Cannot open the page:** confirm the IP printed by the device and ensure
  the computer is on the same network.
- **Upload fails:** check the serial port, bootloader mode, cable, and
  `upload_port` setting.
