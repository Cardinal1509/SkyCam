# SkyCam: 3D Cable-Driven Parallel Robot (CDPR) & Active Pan Tracking Rig

An autonomous, 4-cable suspended parallel robot with a decoupled 1-DOF active pan gimbal and closed-loop computer vision tracking. Built on a \$145 self-funded budget from structural 2x4 lumber, dual ESP32 microcontrollers, four NEMA 17 steppers, and Python OpenCV.

Designed as an autonomous robotic camera operator to track musicians and moving subjects hands-free in a room.

---

## 1. System Overview

```
                                  [ PC Workstation (OpenCV) ]
                                   /                       \
                      Wi-Fi 640x480                         USB Serial (115200)
                     MJPEG Stream @ 30 Hz                  Command: X<dx>,Y<dy>,Z<dz>,A<deg>
                                 /                           \
               [ AI-Thinker ESP32-CAM ]             [ ESP32 DevKit V1 Motion Controller ]
              (OV3660 + Inverted Mount)              (FastAccelStepper Hardware MCPWM/RMT)
                         |                                    |                 \
                  SG90 Micro Servo                     CNC Shield V3         GPIO 13 (50 Hz PWM)
               (Active Pan Gimbal)                   (4x Bolsen A4988)               |
                         |                             |       |        \            |
                 Carriage Payload                   Motor X  Motor Y  Motor Z  Motor A
             (4-Corner Dyneema Winches)            (760, 0)  (0, 0)  (760, 760) (0, 760)
```

The system decouples fast rotational tracking from translational cable motion:
1. **High-Bandwidth Gaze Tracking (30 Hz)**: An inverted ESP32-CAM mounted on a 1-DOF SG90 servo actively yaws to keep the AprilTag centered in the visual frame.
2. **Synchronized 3D Cable Translation (8 Hz)**: The controller projects the target's 3D offset through the camera's live gaze angle into world coordinates, driving four independent NEMA 17 winches via hardware timer step generation.

---

## 2. Technical Specifications

| Parameter | Specification | Engineering Details |
| :--- | :--- | :--- |
| **Workspace Envelope** | $760 \times 760 \times 685.8\text{ mm}$ ($2.5 \times 2.5 \times 2.25\text{ ft}$) | Kiln-dried SPF 2x4 structural perimeter elevated 27" on corner supports |
| **Carriage Dimensions** | $101.6 \times 101.6\text{ mm}$ ($4.0 \times 4.0\text{ in}$) | Plywood plate with 4-corner Dyneema attachments & under-slung ballast |
| **Linear Resolution** | $44.444\text{ steps/mm}$ ($0.0225\text{ mm/step}$) | 36T GT2 aluminum pulley ($72.0\text{ mm}$ circ.) at $1/16$ microstepping |
| **Rigging Line** | $65\text{ lb}$ Hercules UHMWPE Dyneema | 4-strand zero-stretch braid; 4 friction capstan wraps on drums |
| **Fairleads** | 4× HUABAN U604ZZ U-groove bearings | Angled 45° inward, shimmed with washer stacks to match spool height |
| **Max Winch Speed** | $15,500\text{ steps/s}$ ($348.75\text{ mm/s}$) | Generated via `FastAccelStepper` hardware timer interrupts |
| **Max Acceleration** | $35,000\text{ steps/s}^2$ ($787.5\text{ mm/s}^2$) | Smooth trapezoidal velocity ramping |
| **Power Distribution** | $12\text{V } 5\text{A}$ DC Brick $\rightarrow$ Parallel Split | $12\text{V}$ high-voltage motor rail + LM2596 buck converter ($5.05\text{V}$) |
| **Vision Pipeline** | OpenCV `cv2.aruco` (`DICT_APRILTAG_36h11`) | $640 \times 480$ VGA MJPEG stream over HTTP port 80 at 30 FPS |
| **Serial Telemetry** | $115,200\text{ baud}$ bidirectional UART | Custom ASCII protocol (`X<dx>,Y<dy>,Z<dz>,A<deg>\n`) with `[ACK]` |
| **Total Build Cost** | **\$142.10** | Self-funded; 100% off-the-shelf components and basic hand tools |

---

## 3. Kinematics & Multi-Axis Synchronization

### 3.1 4-Corner Cable Length Formulation
Unlike idealized parallel robots that treat the end-effector as an infinitesimal point, SkyCam explicitly accounts for the $101.6\text{ mm}$ square carriage dimensions ($c_w = 101.6\text{ mm}$, $c_{\text{half}} = 50.8\text{ mm}$). 

For each corner $i \in \{X, Y, Z, A\}$ with motor fairlead coordinates $(M_{x,i}, M_{y,i}, Z_{\text{TOP}})$ and carriage corner offsets $(c_{x,i}, c_{y,i}) \in \{\pm 50.8\text{ mm}\}$:

$$L_i(X, Y, Z) = \sqrt{(M_{x,i} - (X + c_{x,i}))^2 + (M_{y,i} - (Y + c_{y,i}))^2 + (Z_{\text{TOP}} - Z)^2}$$

The required step displacement relative to the calibrated home origin is:
$$\Delta\text{Steps}_i = (L_i - L_{\text{home},i}) \times 44.444\text{ steps/mm}$$

### 3.2 Proportional Multi-Axis Speed/Acceleration Scaling
If each stepper ran at a constant default speed, shorter cables would finish moving before longer cables, causing mid-trajectory cable slack and carriage tilting. To guarantee all four winches start, accelerate, coast, and stop at the exact same millisecond:

$$\text{maxDelta} = \max(|\Delta\text{Steps}_0|, |\Delta\text{Steps}_1|, |\Delta\text{Steps}_2|, |\Delta\text{Steps}_3|)$$
$$\text{ratio}_i = \frac{|\Delta\text{Steps}_i|}{\text{maxDelta}}$$
$$\text{Speed}_i = \max(\text{MIN\_SPEED}, \text{MAX\_SPEED} \times \text{ratio}_i)$$
$$\text{Accel}_i = \max(\text{MIN\_ACCEL}, \text{MAX\_ACCEL} \times \text{ratio}_i)$$

---

## 4. Key Electrical & Low-Level Debugging

### 4.1 The Two-Stage Logic Rail & CMOS Threshold Fix
Initial bench tests failed to drive any steppers despite $5.05\text{V}$ measured on the CNC Shield logic rail. Analysis of the A4988 datasheet revealed:
* Minimum logic-HIGH input threshold: $V_{IH(\min)} = 0.7 \times V_{DD}$.
* When $V_{DD} = 5.05\text{V}$, $V_{IH(\min)} = 3.54\text{V}$.
* The 3.3V ESP32 GPIO signals ($3.30\text{V}$) were lower than $3.54\text{V}$, causing the drivers to reject all step pulses.

**Solution**:
1. Routed the LM2596 buck converter ($5.05\text{V}$) into the `ESP32 VIN` pin.
2. Routed the ESP32's onboard LDO-regulated `3.3V` output pin to the CNC Shield logic rail (`SV/GND`), reducing $V_{DD}$ to $3.30\text{V}$.
3. This lowered $V_{IH(\min)}$ to $0.7 \times 3.30\text{V} = 2.31\text{V}$, creating a robust $+0.99\text{V}$ noise margin for the ESP32's $3.3\text{V}$ GPIO outputs.

### 4.2 Ratiometric $V_{\text{ref}}$ Divider Drop
Dropping $V_{DD}$ from $5.05\text{V}$ to $3.30\text{V}$ ratiometrically pulled down the A4988 $V_{\text{ref}}$ trimpot divider from $0.72\text{V}$ to $0.48\text{V}$. Using an EverStart digital multimeter, all four trimpots were re-tuned clockwise back to $0.72\text{V}$ for the target current limit:
$$I_{\text{limit}} = \frac{V_{\text{ref}}}{8 \times R_{CS}} = \frac{0.72\text{V}}{8 \times 0.100\,\Omega} = 0.90\text{ A}$$

### 4.3 Crossed Phase Coils
When the motors vibrated with high holding torque but would not step, resistance probing revealed crossed coil wiring ($7\,\Omega$ between pins 1–3 and 2–4). The XH2.54 connector latches were de-pinned and the middle Blue/Green leads were swapped to restore true phase pairing (Pins 1–2 = Coil A, Pins 3–4 = Coil B).

### 4.4 Custom 512-Byte RAM Block Flashing (`flash_esp32.py`)
Flashing the AI-Thinker ESP32-CAM routinely failed with `0107: Checksum error` due to background Arduino polling (`serial-discovery.exe`) and CH340 UART FIFO buffer overflow under esptool's default 6,144-byte RAM stub blocks. A custom Python script (`flash_esp32.py`) was written to force `esp.ESP_RAM_BLOCK = 512` at 230,400 baud, ensuring 100% reliable flashing.

---

## 5. Computer Vision & Gaze Projection

### 5.1 Chiral Inversion Bug
Mounting the ESP32-CAM upside-down required flipping the image. However, an initial double-mirroring flipped the chirality of the asymmetric $6 \times 6$ `tag36h11` matrix, preventing OpenCV `ArucoDetector` from decoding the checksum. The fix was applying a single vertical flip (`cv2.flip(frame, 0)`), which preserves matrix chirality.

### 5.2 2D Gaze to 3D World Transformation
The active SG90 pan servo tracks the target at angle $\theta$ (degrees). The 2D visual errors are converted to 3D displacement vectors:
* Lateral Error: $e_x = c_x - 320$
* Depth Error: $e_{\text{depth}} = 55 - \text{tag\_width}$
* Vertical Error: $e_z = c_y - 240$

```python
theta_rad = math.radians(current_servo_angle)
fwd_x = -math.sin(theta_rad)
fwd_y =  math.cos(theta_rad)
right_x =  math.cos(theta_rad)
right_y =  math.sin(theta_rad)

target_world_x = current_x + (fwd_x * depth_lead + right_x * lateral_lead)
target_world_y = current_y + (fwd_y * depth_lead + right_y * lateral_lead)
target_world_z = current_z + vertical_lead
```

---

## 6. Repository Structure

```
SkyCam/
├── README.md                      # Complete system documentation
├── .gitignore                     # Git ignore rules
├── skycam_controller.py           # Main OpenCV 36h11 tracker + 3D gaze-to-world projector
├── stream_worker.py               # Zero-latency daemon thread MJPEG stream grabber
├── track_test.py                  # Standalone OpenCV AprilTag detection benchmark
├── camtest.py                     # Stream FPS & latency tester
├── cam_program/
│   ├── cam_program.ino            # AI-Thinker ESP32-CAM VGA MJPEG HTTP streaming firmware
│   └── flash_esp32.py             # Custom 512B RAM block esptool flashing utility
├── motor_firmware/
│   └── motor_firmware.ino         # Production ESP32 4-axis FastAccelStepper + 50Hz LEDC PWM firmware
├── demo3d/
│   └── demo3d.ino                 # Autonomous 3D perimeter square patrol + vertical Z-lift firmware
└── motor_test/
    └── motor_test.ino             # 4-axis stepper bench validation script (15.5 kHz)
```

---

## 7. How to Run

1. **Flash Firmware**:
   - Flash `motor_firmware/motor_firmware.ino` to the ESP32 DevKit V1 using Arduino IDE (requires `FastAccelStepper` library).
   - Flash `cam_program/cam_program.ino` to the AI-Thinker ESP32-CAM using `python cam_program/flash_esp32.py`.
2. **Start Video Stream**:
   - Power the ESP32-CAM and confirm assigned IP on Wi-Fi (e.g., `http://192.168.1.26/stream`).
3. **Launch Controller**:
   ```bash
   pip install opencv-python pyserial numpy
   python skycam_controller.py
   ```
4. **Operation**:
   - Press `H` to tare/home the origin $(380, 380, 480)\text{ mm}$.
   - Press `T` to toggle autonomous closed-loop visual tracking.
   - Press `Q` to park and exit.

---

## 8. Assistive AI & Tool Disclosures
In accordance with MIT Maker Portfolio guidelines:
* **Hardware & Testing**: Structural frame fabrication, mechanical alignment, wiring, and digital multimeter measurements were executed 100% by hand.
* **Assistive AI Use**: An LLM was used as an interactive engineering sounding board to verify structural deflection calculations, troubleshoot the CMOS $V_{IH(\min)}$ threshold mismatch, co-develop `FastAccelStepper` and OpenCV control loops, diagnose UART FIFO overflow during camera flashing, and derive the 2D-gaze-to-3D-world matrix rotation.
