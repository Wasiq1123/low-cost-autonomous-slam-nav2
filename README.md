# Low-Cost Autonomous Mapping & Navigation — Phone Camera + $5 Microcontroller, No RGB-D Sensor, No Raspberry Pi

**A full SLAM + Nav2 mobile robot pipeline built without either of the two components that normally dominate the cost of a project like this — an RGB-D depth camera and an onboard compute board (Raspberry Pi/Jetson) — replaced with a phone camera, a fine-tuned monocular depth model, and a $5 ESP32.**

---

## Problem statement

A typical autonomous mapping robot built the "standard" way needs:
- An RGB-D depth camera (Intel RealSense D435i or similar) — **$200–$350**
- An onboard compute board to run everything (Raspberry Pi 4/5 with adequate cooling, or a Jetson) — **$50–$200+**

For a student project — or for anyone trying to replicate this without a lab budget — that's the single biggest barrier to entry, often costing more than the robot chassis and motors combined.

**This project removes both:**
1. **No depth camera.** An Android phone's existing RGB camera, streamed over WiFi via [DroidCam](https://www.dev47apps.com/), replaces the RGB-D sensor. Metric depth is estimated from that RGB stream with the fine-tuned YOLO26-Depth model from this project's [depth estimation work](#related-repos) — the same phone most people already own does the job a $200+ sensor would have.
2. **No Raspberry Pi.** All the heavy compute — depth estimation, RTAB-Map SLAM, Nav2 — runs on a laptop, which most people already have too. The robot itself only needs a **$5 ESP32** microcontroller to read wheel encoders and drive motor PWM, talking to the laptop wirelessly over WiFi via a custom `ros2_control` hardware interface. No onboard Linux board, no SD card, no thermal throttling under load.

| | Standard approach | This project |
|---|---|---|
| Depth sensing | RGB-D camera, ~$200–350 | Phone (already owned) + DroidCam, effectively $0 |
| Onboard compute | Raspberry Pi/Jetson, ~$50–200+ | ESP32, ~$5 |
| Where compute actually happens | On the robot | On a laptop, wirelessly |

---

## System architecture

```
Android phone (DroidCam app)
        │  WiFi
        ▼
v4l2loopback virtual camera (/dev/video0)
        │
   ros2 run depth droid          ──▶  /camera/color/image_raw, /camera/color/camera_info
        │
   ros2 run depth new_rtabmap_depth  ──▶  /camera/depth/image_raw, /camera/depth/camera_info
        │                                (OpenVINO YOLO26-Depth, fine-tuned — see Related repos)
        │
        ├───────────────────────────────────────────┐
        │                                            │
   ros2_control (diff hardware interface)      RTAB-Map SLAM
        │  WiFi (TCP, port 80)                       │  (droidcam_rtabmap.launch.py)
        ▼                                             ▼
   ESP32 ($5) — final_code_1.ino                    Nav2
   - reads left/right quadrature encoders     (navigation.launch.py)
   - drives motor PWM
   - reports odometry over WiFi
        │
   EKF (robot_localization) fuses
   wheel odometry + IMU  ──▶  /odom
```

Everything left of the ESP32 box runs on your laptop. The robot itself is just wheels, motors, an IMU, and the ESP32 — no onboard Linux, no camera mount for a depth sensor, nothing else to power or cool.

---

## Repo layout

```
depth_models/src/depth/          # Depth pipeline nodes                    ← this project
├── depth/rtabmap_droidcam.py    #   `droid` executable — DroidCam → ROS 2 image topic
└── depth/new_rtabmap_depth.py   #   `new_rtabmap_depth` executable — OpenVINO depth inference

my_team/src/
├── megabot_nav/                 # RTAB-Map + Nav2 bring-up                ← this project
│   ├── launch/droidcam_rtabmap.launch.py
│   ├── launch/navigation.launch.py
│   ├── launch/localization.launch.py
│   ├── map/                     #   saved occupancy grid
│   └── param/mws_params.yaml    #   Nav2 parameters
└── bt_ros2/                     # Behavior Tree layer over Nav2 (ADLINK NeuronBot2, adapted)

ros2_control/src/
├── diff/                        # Custom hardware interface package      ← this project
│   ├── src/diffbot_system.cpp   #   WiFi client — talks to the ESP32
│   ├── include/diff/diffbot_system.hpp
│   ├── launch/control.launch.py #   dual-mode: real hardware or Gazebo sim, same URDF
│   ├── urdf/mws_control.urdf
│   ├── config/controllers.yaml, ekf.yaml, madgwick.yaml
│   └── worlds/depot.sdf
├── serial/, serial_ros/         # Vendored serial communication packages (wjwwood/serial)
└── transport_drivers/           # Vendored ROS 2 transport drivers (asio, serial_driver, udp_driver)

esp32_diffbot_firmware.ino       # ESP32 firmware — WiFi server, encoders, motor PWM  ← this project
```

`build/`, `install/`, `log/`, and the venv under `depth_models/src/depth/depth/venv/` are present in this export but shouldn't be committed — same as every other repo in this project, add a `.gitignore` covering `venv/`, `**/venv/`, `build/`, `install/`, `log/` before pushing.

---

## The ESP32 side — how a $5 board replaces a Raspberry Pi

`esp32_diffbot_firmware.ino` runs a `WiFiServer` on port 80. `diffbot_system.cpp` (the `ros2_control` hardware interface, running on your laptop) connects to it as a TCP client — `esp32_ip`/`esp32_port` are set in `mws_control.urdf`'s `<hardware>` block (defaults: `192.168.43.55:80`, update to match your ESP32's actual IP once it connects to your WiFi).

What the firmware actually does:
- Reads left/right quadrature encoders via interrupt service routines (`leftEncoderISR`/`rightEncoderISR`)
- Drives left/right motor PWM (`analogWrite`) from commands received over the WiFi connection
- Reports encoder counts back, which `diffbot_system.cpp` turns into wheel odometry

That's the entire onboard workload — everything else (depth estimation, SLAM, path planning) happens on the laptop across the WiFi link. This is what makes the $5 figure real: the ESP32 only needs to do fast, simple I/O, not run any part of the perception or planning stack.

**Before flashing:** the sketch has `WIFI_SSID`/`WIFI_PASSWORD` and pin assignments (`LEFT_MOTOR_RPWM`, `RIGHT_ENCODER_PIN`, etc.) near the top — set these to match your actual wiring and network before uploading via the Arduino IDE.

---

## Setup

### 1. Phone camera via DroidCam

Install the [DroidCam](https://www.dev47apps.com/) app on your Android phone and the matching client on your laptop, both on the same WiFi network.

```bash
# Is v4l2loopback loaded?
lsmod | grep v4l2loopback

# If not, load it manually:
sudo modprobe v4l2loopback exclusive_caps=1 card_label="DroidCam"

# Confirm the device shows up:
ls /dev/video*
```

Then, with the DroidCam app open on your phone (note the IP address it shows):

```bash
droidcam-cli <your-phone's-wifi-ip> 4747
```

Verify the stream is actually coming through before starting ROS:

```bash
ffplay /dev/video0
```

### 2. ESP32

Flash `esp32_diffbot_firmware.ino` (edit `WIFI_SSID`/`WIFI_PASSWORD` and pin definitions first), power it on, and note the IP address it prints over serial — update `esp32_ip` in `mws_control.urdf` to match if it differs from the default.

### 3. Build the ROS 2 workspace

Standard `colcon build` once all three packages (`depth`, `megabot_nav`, `diff` + the vendored serial/transport_drivers packages) are in your workspace `src/`.

---

## Usage

Run each in its own terminal, in order:

```bash
# 1. Camera + depth estimation
ros2 run depth droid
ros2 run depth new_rtabmap_depth

# 2. Robot hardware interface — real hardware, not simulation
ros2 launch diff control.launch.py use_sim_time:=false use_ekf:=true use_rviz:=true

# 3. RTAB-Map SLAM (mapping mode; add localization:=true once you have a saved map)
ros2 launch megabot_nav droidcam_rtabmap.launch.py

# 4. Nav2
ros2 launch megabot_nav navigation.launch.py use_sim_time:=false
```

---

## Setup notes

- Camera intrinsics in `rtabmap_droidcam.py` (`fx=401.0, fy=403.5`) are estimated for one specific phone (Tecno Spark) — recalibrate for your own camera.
- `new_rtabmap_depth.py`'s `MODEL_PATH` points at a local path. Download the checkpoint from Hugging Face and point `MODEL_PATH` at wherever you save it:
  ```bash
  pip install -U "huggingface_hub[cli]"
  huggingface-cli download WasiqSaleem/low-cost-autonomous-slam-nav2 --local-dir ./checkpoints
  ```
- **`scale_factor = 1.0`** in the same file is a placeholder. `measure_depth_scale.py` from [realsense-lab-dataset-tools](#related-repos) is built for exactly this — run it against the live `/camera/depth/image_raw` topic and update `scale_factor` before trusting absolute distances from this pipeline.

## Related repos

* Depth model fine-tuning: [depth-estimation-finetuning-yolo26-dav2](https://github.com/Wasiq1123/depth-estimation-finetuning-yolo26-dav2) — trains the checkpoint `new_rtabmap_depth.py` runs
* Deployment optimization: [depth-estimation-openvino-quantization](https://github.com/Wasiq1123/depth-estimation-openvino-quantization) — the OpenVINO conversion this pipeline depends on
* Offline evaluation: [depth-estimation-eval-toolkit](https://github.com/Wasiq1123/depth-estimation-eval-toolkit) — how the depth checkpoint's accuracy was benchmarked before deployment here
* Dataset capture: [realsense-lab-dataset-tools](https://github.com/Wasiq1123/realsense-lab-dataset-tools) — `measure_depth_scale.py` is the tool for calibrating `scale_factor` above
* ROS 2 depth packages: [Fine-Tuned-Depth-Estimation-for-ROS-2](https://github.com/Wasiq1123/Fine-Tuned-Depth-Estimation-for-ROS-2), [Multi-Model-Monocular-Depth-Estimation-for-ROS-2](https://github.com/Wasiq1123/Multi-Model-Monocular-Depth-Estimation-for-ROS-2)
* Nav2/SLAM reference work: [autonomous-wheelchair-slam-nav2](https://github.com/Wasiq1123/autonomous-wheelchair-slam-nav2), [alif-racer-ros2-nav2-slam](https://github.com/Wasiq1123/alif-racer-ros2-nav2-slam) — earlier projects this one's `megabot_nav`/`bt_ros2` structure builds on

## Acknowledgments

`bt_ros2` is adapted from ADLINK Technology's [BT_ros2](https://github.com/Adlink-ROS/BT_ros2). `serial`/`serial_ros`/`transport_drivers` are vendored open-source ROS 2 communication packages. DroidCam is developed by [Dev47Apps](https://www.dev47apps.com/).

## License

Apache-2.0 — for the original packages (`depth`, `megabot_nav`, `diff`, the ESP32 firmware), matching the license already used across every other repo in this project (fine-tuning, quantization, eval toolkit, dataset tools, both ROS depth packages), so the whole body of work is licensed consistently. Vendored packages retain their original upstream licenses.
