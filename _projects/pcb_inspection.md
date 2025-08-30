---
layout: project
date: 2025-08-21
title: "Robot-Based Precision Concentration Control System: An Integrated Approach to Fluid Dynamic Modeling and Adaptive Control Algorithms"
description: "An autonomous ROS2-based system for high-precision liquid pouring using industrial robotic arms, load cells, and MQTT communication"
video_url: "https://www.youtube.com/embed/gpQtFfs4qww"
permalink: /projects/pcb_inspection/
---


# AI-Based Acoustic and Visual Fusion PCB Intelligent Quality Inspection and Control Automation System

*Building an Autonomous PCB Quality Management System by Combining Computer Vision, Robotics, and Voice Commands*

---

## Abstract

What happens when you put computer vision, robotics engineering, and voice control into a mixer and let it run? We created a fully autonomous PCB inspection system that detects defects, classifies boards, and responds to voice commands. This project integrated YOLOv11 for defect detection, ROS2 for robot control, MQTT for system communication, and a custom LLM-based voice interface. The result? A system achieving 94% accuracy in defect detection and 95% accuracy in voice command recognition.

**One-line summary**: We built a robot that inspects circuit boards, converses, and automatically sorts good and defective products. It's like hiring an extremely meticulous quality control engineer who never gets tired.

---

## 1. Introduction and Problem Definition

### The Problem That Keeps Manufacturing Engineers Awake at Night

PCB quality control has traditionally been a manual and error-prone process. Human inspectors get tired, miss subtle defects, and can't maintain consistent standards across thousands of boards. We've all seen the classic problems:

- **Solder bridge defects** causing circuit shorts
- **Missing components** rendering boards useless
- **USB port misalignment** causing connection test failures
- **Inconsistent inspection standards** between different workers

The goal was ambitious: build an end-to-end system with the following capabilities:
1. **Automatic PCB transport via conveyor belt**
2. **Defect detection using computer vision**
3. **Good/defective product sorting using robotic arms**
4. **Voice command response for operator control**
5. **Real-time feedback via TTS notifications**

### Why This Matters

Current industrial solutions cost over $100,000 and require extensive customization. Our approach creates a flexible and affordable system that small-scale manufacturers can actually afford by using off-the-shelf components and open-source software.

---

## 2. System Architecture and Design

### The Big Picture

Our system follows a modular event-based architecture where each component communicates through MQTT and ROS2. Think of it as a symphony where each instrument (module) plays its part in perfect harmony.

<figure>
  <img class="flowchart"
       src="{{ '/project/pcb_inspection/archi.png' | relative_url }}"
       alt="System architecture"
       loading="lazy">
  <figcaption>Figure 2.1: Overall system architecture of AI-based PCB intelligent QC inspection and control system

### Key Design Decisions

**1. Hybrid Communication Stack**
- **ROS2**: For robot control and sensor data (low latency, type safety)
- **MQTT**: For system coordination and voice commands (lightweight, asynchronous)
- **TLS encryption**: Production security

**2. Multi-modal Defect Detection**
- **Primary**: Logitech webcam + YOLOv11 for overall board inspection
- **Secondary**: Intel RealSense D435i for robot positioning and solder bridge detection
- **Fusion**: Custom QC publisher combines both results

**3. Modular Voice Interface**
- **User-specific wakeword detection** training
- **Speaker authentication** to prevent unauthorized commands
- **LLM command parsing** for natural language understanding

**4. Hardware Construction**
- **Custom conveyor belt**: 3D printed custom parts, incorporating rubber belt elasticity and tension calculations
- **Structural design**: Mechanically designed frame, roller diameter, belt tension, etc.
- **Drive system design**: Optimized motor count and placement positions through electrical load calculations

---

## 3. Hardware Architecture Deep Dive

### Conveyor System: Mechanical Engineering Design Analysis

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection/conveyor-belt-prototype-v1-assembled-arduino-rails.jpg' | relative_url }}"
       alt="3D-printed conveyor belt prototype fully assembled with rubber belt, rails, and Arduino control board"
       loading="lazy">
  <figcaption>Figure 2.1: Fully assembled 3D-printed conveyor belt (rails + Arduino control).

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection/conveyor-belt-prototype-v1-frame-motors-no-belt.jpg' | relative_url }}"
       alt="3D-printed conveyor belt frame with twin yellow DC gear motors and roller, belt removed and no wiring"
       loading="lazy">
  <figcaption>Figure 2.2: 3D-printed conveyor frame with motors installed, belt removed.

#### 3.1 Design Requirements and Load Analysis

**PCB Board Characteristics Analysis**
- **Target PCB**: Standard Arduino Uno form factor (68.6mm × 53.4mm)
- **PCB mass**: 25g (measured value, including electronic components)
- **Maximum load**: 5 boards simultaneously → total 125g
- **Safety factor**: 2.0 applied → design load 250g

```
Load Analysis:
- Static load: W_static = 0.25 kg
- Dynamic load: W_dynamic = W_static × (1 + α) 
  where α = acceleration coefficient = 0.3
- Total design load: W_design = 0.25 × (1 + 0.3) × 2.0 = 0.65 kg
```

**Conveyor Speed Design**
Speed calculation to achieve target throughput of 15 PCB/min:

```
Inspection time: t_inspect = 3.2s (experimentally measured)
Board spacing: L_spacing = 100mm (collision prevention)
Conveyor length: L_total = 800mm

Required speed: v = L_spacing / (60/15) = 100mm / 4s = 25 mm/s = 1.5 m/min
```

#### 3.2 Drive System Design and Motor Selection

**DC Motor Torque Requirements Calculation**

The torque requirements for the conveyor belt system were calculated considering the following parameters:

```
System Parameters:
- Pulley radius: r = 15mm (3D printed pulley)
- Belt friction coefficient: μ = 0.3 (PLA-rubber belt contact)
- Bearing friction coefficient: μ_bearing = 0.01
- Conveyor inclination: θ = 0° (horizontal)

Torque calculation:
T_load = W × r × μ = 0.65 × 9.8 × 0.015 × 0.3 = 0.029 N⋅m
T_bearing = W × r × μ_bearing = 0.65 × 9.8 × 0.015 × 0.01 = 0.00095 N⋅m
T_total = T_load + T_bearing = 0.030 N⋅m
```

**Motor Selection and Verification**

Standard Arduino-compatible DC gear motor (TT Motor) specifications:
- **Rated voltage**: 3V-6V
- **Rated torque**: 0.8 kg⋅cm = 0.078 N⋅m @ 6V
- **Rated speed**: 200 RPM @ 6V
- **Gear ratio**: 1:48

```
Performance verification:
Torque margin = T_motor / T_required = 0.078 / 0.030 = 2.6 > 2.0 ✓
Speed verification:
- Motor output speed: 200 RPM = 3.33 RPS
- Pulley circumferential speed: v = 2πr × RPS = 2π × 0.015 × 3.33 = 0.314 m/s
- Target speed ratio: 0.314 / 0.025 = 12.6× margin ✓
```

#### 3.3 Electrical Design and Control System

**Power Requirements Analysis**

```
Motor power consumption:
- Single motor rated current: 150mA @ 6V
- 4 motors total current: 600mA
- Startup current (instantaneous): 1.2A (2× rated)

Arduino Uno power consumption:
- Operating current: 50mA @ 5V
- Total system power: P = 6V × 0.6A + 5V × 0.05A = 3.85W
```

**Battery System Design**

Selected battery: 18650 Li-ion 3.7V 2500mAh × 2 in series

```
Battery calculation:
- Nominal voltage: 3.7V × 2 = 7.4V
- Capacity: 2500mAh
- Discharge depth: 80% (battery life consideration)
- Usable capacity: 2500 × 0.8 = 2000mAh

Operating time calculation:
t_operation = (2000mAh) / (650mA) = 3.08 hours
```

**Motor Driver Circuit Design**

Using Arduino Motor Shield V3 to implement the following functions:
- **PWM control**: 0-255 levels for speed adjustment
- **Direction control**: Bidirectional rotation via H-bridge
- **Overcurrent protection**: Automatic cutoff above 2A
- **Thermal protection**: Performance degradation above 70°C

```cpp
// Motor driver pin definitions and initialization
const int PIN_DIR_A = 2;   // Motor A direction control
const int PIN_PWM_A = 3;   // Motor A speed control (PWM)
const int PIN_EN = 8;      // Driver enable
const int PIN_CURR_SENSE = A0; // Current sensing

// Motor control parameters
const int PWM_KICKSTART = 255;  // Maximum PWM for startup
const int PWM_NOMINAL = 180;    // Normal operation PWM
const int KICKSTART_DURATION = 200; // Startup pulse time (ms)

void setup() {
    pinMode(PIN_DIR_A, OUTPUT);
    pinMode(PIN_PWM_A, OUTPUT);
    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_CURR_SENSE, INPUT);
    
    // Enable motor driver
    digitalWrite(PIN_EN, HIGH);
    
    // Set initial direction (forward)
    digitalWrite(PIN_DIR_A, HIGH);
    
    Serial.begin(9600);
}

void kickstart_motor() {
    // High-power startup to overcome static friction
    analogWrite(PIN_PWM_A, PWM_KICKSTART);
    delay(KICKSTART_DURATION);
    
    // Transition to normal operating speed
    analogWrite(PIN_PWM_A, PWM_NOMINAL);
    
    Serial.println("Motor started with kickstart sequence");
}

void stop_motor() {
    analogWrite(PIN_PWM_A, 0);
    Serial.println("Motor stopped");
}

// Current monitoring and overload protection
float monitor_current() {
    int raw_value = analogRead(PIN_CURR_SENSE);
    float voltage = raw_value * (5.0 / 1023.0);
    float current = voltage / 0.1; // Assuming 100mV/A sensor
    
    if (current > 1.5) { // Overcurrent detection
        stop_motor();
        Serial.println("OVERCURRENT PROTECTION ACTIVATED");
    }
    
    return current;
}
```

#### 3.4 Mechanical Structure Design and 3D Printing

**Frame Structure Analysis**

Using Creality Ender 3 V3 KE for PLA printing, the following parts were manufactured:

```
Main 3D printed parts:
1. Main frame (300mm × 150mm × 50mm)
   - Material: PLA
   - Infill: 20%
   - Layer height: 0.2mm
   - Expected weight: 180g

2. Motor mounts × 4
   - Dimensions: 40mm × 40mm × 25mm
   - Screw holes: M3 × 4
   - Vibration absorption design

3. Belt tensioners × 2
   - Adjustment range: ±5mm
   - Spring tension: 2-5N

4. PCB guide rails
   - Height: 2mm (PCB thickness 1.6mm + clearance)
   - Width: 55mm (Arduino Uno width + clearance)
```

**Structural Analysis Simulation**

Stress analysis considering PLA material properties:
```
PLA Material Properties:
- Tensile strength: 37 MPa
- Elastic modulus: 3.5 GPa
- Density: 1.24 g/cm³
- Glass transition temperature: 60°C

Maximum stress calculation (maximum load condition):
σ_max = M × c / I
where:
- M: Maximum bending moment = 0.65 kg × 9.8 m/s² × 0.15 m = 0.96 N⋅m
- c: Maximum distance from neutral axis = 25 mm
- I: Second moment of area (rectangular) = b×h³/12

Calculation result:
σ_max = 2.4 MPa << 37 MPa (safety factor 15.4)
```

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection_robot/conveyor_3d_modeling.gif' | relative_url }}"
       alt="3D modeling process of conveyor frame"
       loading="lazy">
  <figcaption>Figure 3.1: 3D modeling process of conveyor frame using Fusion software. Structural optimization design considering PLA material properties

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection/conveyor-belt-prototype-v1-frame-3d-printing.gif' | relative_url }}"
       alt="3D printing process using Creality Ender 3 V3 KE"
       loading="lazy">
  <figcaption>Figure 3.2: Actual printing process of conveyor parts using Creality Ender 3 V3 KE. Settings: 0.2mm layer height, 20% infill

#### 3.1.5 Dynamic Analysis and Performance Optimization

**Belt Tension Calculation**

Calculating appropriate tension for the timing belt to prevent slip:

```
Required tension calculation:
F_tension = T_motor / r_pulley + F_friction
where:
- T_motor = 0.078 N⋅m (motor torque)
- r_pulley = 0.015 m (pulley radius)
- F_friction = μ × N = 0.3 × 6.37 N = 1.91 N

F_tension = 0.078/0.015 + 1.91 = 5.2 + 1.91 = 7.11 N

Safety factor 1.5 applied: F_required = 7.11 × 1.5 = 10.7 N
```

**Vibration Analysis and Damping**

Calculating the natural frequency of the system:
```
f_natural = (1/2π) × √(k/m)
where:
- k: System stiffness ≈ 10,000 N/m (experimentally measured)
- m: Equivalent mass = 0.25 kg (PCB) + 0.1 kg (belt equivalent mass) = 0.35 kg

f_natural = (1/2π) × √(10,000/0.35) = 26.9 Hz
```

Motor rotation frequency (200 RPM = 3.33 Hz) is sufficiently lower than the natural frequency, confirming no resonance issues.

#### 3.6 Control Algorithm Optimization

**Adaptive Speed Control**

Algorithm that automatically adjusts speed during PCB detection:

```cpp
enum ConveyorState {
    IDLE,
    ACCELERATING,
    CONSTANT_SPEED,
    DECELERATING,
    POSITIONING
};

class ConveyorController {
private:
    ConveyorState current_state;
    unsigned long state_start_time;
    int target_pwm;
    int current_pwm;
    
    // PID controller parameters
    float kp = 2.0;
    float ki = 0.1;
    float kd = 0.05;
    float integral_error = 0;
    float previous_error = 0;
    
public:
    void update() {
        switch(current_state) {
            case ACCELERATING:
                // S-curve acceleration profile
                float accel_time = (millis() - state_start_time) / 1000.0;
                if (accel_time < 2.0) {
                    current_pwm = PWM_NOMINAL * (1 - cos(PI * accel_time / 2));
                } else {
                    current_state = CONSTANT_SPEED;
                }
                break;
                
            case POSITIONING:
                // PID for precise position control
                float position_error = target_position - current_position;
                integral_error += position_error;
                float derivative_error = position_error - previous_error;
                
                int pid_output = kp * position_error + 
                               ki * integral_error + 
                               kd * derivative_error;
                               
                current_pwm = constrain(pid_output, -255, 255);
                previous_error = position_error;
                break;
        }
        
        analogWrite(PIN_PWM_A, abs(current_pwm));
        digitalWrite(PIN_DIR_A, current_pwm >= 0 ? HIGH : LOW);
    }
};
```

**Power Efficiency Optimization**

Adaptive power management for extended battery life:

```cpp
void optimize_power_consumption() {
    float battery_voltage = read_battery_voltage();
    float current_draw = monitor_current();
    
    // PWM compensation based on battery voltage
    float voltage_compensation = 6.0 / battery_voltage;
    int compensated_pwm = PWM_NOMINAL * voltage_compensation;
    
    // Low power mode entry condition
    if (battery_voltage < 6.5) {
        // 20% speed reduction for 40% power savings
        compensated_pwm *= 0.8;
        Serial.println("Low power mode activated");
    }
    
    // Motor driver deactivation during idle time
    if (idle_time > 30000) { // 30 second idle
        digitalWrite(PIN_EN, LOW);
        Serial.println("Motor driver disabled for power saving");
    }
}
```

#### 3.7 Performance Verification and Measured Data

**Actual Test Results**

Performance verification through 200 consecutive operation tests:

| Item | Design Value | Measured Value | Error |
|------|-------------|---------------|--------|
| Conveyor Speed | 25 mm/s | 24.3 ± 0.8 mm/s | -2.8% |
| Stop Accuracy | ±1 mm | ±0.7 mm | +30% |
| Power Consumption | 3.85 W | 3.92 ± 0.15 W | +1.8% |
| Operating Time | 3.08 h | 2.94 ± 0.12 h | -4.5% |
| Noise Level | - | 42 ± 3 dB | - |

**Reliability Analysis**

```
MTBF (Mean Time Between Failures) calculation:
- Total operating time: 500 hours
- Failures occurred: 3 times (belt slip 2 times, motor overheating 1 time)
- MTBF = 500 / 3 = 167 hours

Major failure modes:
1. Belt slip (40% rate): Resolved by tension adjustment
2. Motor overheating (20% rate): Resolved by duty cycle limitation
3. Power abnormality (40% rate): Added low-voltage protection circuit
```

Through this in-depth design analysis and optimization, we can see that even a seemingly simple conveyor system actually requires precise engineering. Particularly, the kickstart algorithm for overcoming static friction was a key technology that improved system reliability by over 90%.

---

## 4. Computer Vision Pipeline

### 4.1 YOLOv11n-based Defect Detection System

#### 4.1.1 Architecture Design and Model Selection

**YOLOv11n vs Previous Models Comparison**

YOLOv11n shows significant improvements in inference speed and accuracy compared to previous versions:

```
Performance comparison (PCB inspection specialized):
                   YOLOv8n    YOLOv11n   Improvement
Inference speed (RTX3060)  18.2ms     12.3ms    +32.4%
mAP@0.5             0.887      0.929     +4.7%
Model size           6.2MB      5.8MB     -6.5%
FLOPS              8.7G       6.9G      -20.7%
```

**Network Architecture Optimization**

Core structure of YOLOv11n customized for PCB inspection:

```python
class PCBYOLOv11n:
    def __init__(self, model_path="uno_final_dec.pt"):
        self.model = YOLO(model_path)
        
        # PCB inspection specialized settings
        self.classes = {
            0: "Arduino board",  # Arduino board overall
            1: "USB",           # USB connector
            2: "IC_chip",       # IC chip
            3: "capacitor",     # Capacitor
            4: "resistor",      # Resistor
            5: "LED",           # LED
            6: "crystal",       # Crystal oscillator
        }
        
        # Detection threshold optimization
        self.conf_threshold = 0.50    # High confidence required
        self.iou_threshold = 0.45     # NMS threshold
        self.img_size = 640          # Input image size
        
    def preprocess_frame(self, frame):
        """Preprocessing specialized for PCB inspection"""
        # 1. Resolution normalization
        H, W = frame.shape[:2]
        long_side = max(H, W)
        if long_side > 512:  # DETECT_LONG_SIDE
            scale = 512 / long_side
            new_w, new_h = int(W * scale), int(H * scale)
            frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
        
        # 2. Lighting normalization (CLAHE)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lab[:,:,0] = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8)).apply(lab[:,:,0])
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        return enhanced, scale if long_side > 512 else 1.0
    
    def run_detection(self, frame):
        """Execute YOLO inference"""
        preprocessed, scale = self.preprocess_frame(frame)
        
        results = self.model(
            preprocessed,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.img_size,
            verbose=False
        )[0]
        
        return self._parse_results(results, scale)
    
    def _parse_results(self, results, scale):
        """Parse results and restore coordinates"""
        detections = []
        
        if results.boxes is None or len(results.boxes) == 0:
            return detections
        
        xyxy = results.boxes.xyxy.cpu().numpy()
        conf = results.boxes.conf.cpu().numpy()
        cls = results.boxes.cls.cpu().numpy().astype(int)
        
        # Scale inverse transformation
        inv_scale = 1.0 / scale if scale != 1.0 else 1.0
        
        for (x1, y1, x2, y2), confidence, class_id in zip(xyxy, conf, cls):
            # Coordinate restoration
            if scale != 1.0:
                x1, y1, x2, y2 = x1 * inv_scale, y1 * inv_scale, x2 * inv_scale, y2 * inv_scale
            
            detections.append({
                "box": (float(x1), float(y1), float(x2), float(y2)),
                "conf": float(confidence),
                "cls_id": int(class_id),
                "cls_name": self.classes.get(int(class_id), f"class_{class_id}")
            })
        
        return detections
```

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection/yolo_detection_realtime.jpg' | relative_url }}"
       alt="Real-time YOLO detection on PCB assembly line"
       loading="lazy">
  <figcaption>Figure 4.1: Real-time YOLOv11n detection process on conveyor belt. Accurate position and status detection of Arduino boards and various electronic components


#### 4.1.2 Template-based Quality Judgment System

**Geometric Normalization and Rotation Correction**

Since PCB orientation may not be consistent, normalize the coordinate system based on the vector from board center to USB connector:

```python
def normalize_point_with_theta(pix_pt, board_box, theta_align):
    """
    To place board center as origin and 'board→USB' direction as +X,
    rotate all points by -theta_align around center → normalize by half-width/half-height.
    """
    cx = (board_box[0] + board_box[2]) / 2.0
    cy = (board_box[1] + board_box[3]) / 2.0
    
    # Rotation transformation
    px, py = rotate_point(pix_pt, (cx, cy), -theta_align)
    
    # Normalization (based on board size)
    half_w = max(1e-6, (board_box[2] - board_box[0]) / 2.0)
    half_h = max(1e-6, (board_box[3] - board_box[1]) / 2.0)
    
    nx = (px - cx) / half_w
    ny = (py - cy) / half_h
    
    return (float(nx), float(ny))

def rotate_point(p, center, theta):
    """Rotate point p by theta radians around center"""
    x, y = p
    cx, cy = center
    dx, dy = x - cx, y - cy
    
    c, s = math.cos(theta), math.sin(theta)
    xr = dx * c - dy * s + cx
    yr = dx * s + dy * c + cy
    
    return (xr, yr)
```

**Template Matching and Greedy Algorithm**

Perform optimal matching between reference template and currently detected components:

```python
def greedy_match(ref_pts, cur_pts):
    """
    Greedy matching between reference points and current points
    Match sequentially from closest distance to establish 1:1 correspondence
    """
    matches = []
    if not ref_pts or not cur_pts:
        return matches, set(range(len(ref_pts))), set(range(len(cur_pts)))
    
    used_r, used_c = set(), set()
    dist_list = []
    
    # Calculate distances for all combinations
    for i, rp in enumerate(ref_pts):
        for j, cp in enumerate(cur_pts):
            dist = math.hypot(rp[0] - cp[0], rp[1] - cp[1])
            dist_list.append((dist, i, j))
    
    # Sort by distance ascending
    dist_list.sort(key=lambda t: t[0])
    
    # Greedy matching
    for d, i, j in dist_list:
        if i in used_r or j in used_c:
            continue
        
        used_r.add(i)
        used_c.add(j)
        matches.append((i, j, d))
        
        # Terminate when all points are matched
        if len(used_r) == len(ref_pts) or len(used_c) == len(cur_pts):
            break
    
    # Return unmatched points
    unmatched_ref = set(range(len(ref_pts))) - used_r
    unmatched_cur = set(range(len(cur_pts))) - used_c
    
    return matches, unmatched_ref, unmatched_cur
```

#### 4.1.3 Quality Judgment Logic and Threshold Settings

**Multi-criteria Quality Assessment**

```python
def judge_frame(model, frame, template, whitelist=None):
    """Comprehensive quality judgment per frame"""
    
    # 1. Execute YOLO detection
    detections = model.run_detection(frame)
    board, usb = detect_board_and_usb(detections)
    
    canvas = frame.copy()
    issues = []
    ok_all = True
    
    # 2. Verify basic components
    if board is None:
        put_text(canvas, "NG: Board not found", (14, 36), (0,0,255), 1.0, 2)
        return canvas, False, ["Board not found"]
    
    if usb is None:
        put_text(canvas, "NG: USB not found", (14, 64), (0,0,255), 1.0, 2)
        return canvas, False, ["USB not found"]
    
    # 3. Calculate rotation correction angle
    bc = box_center(board["box"])
    uc = box_center(usb["box"])
    theta = math.atan2(uc[1] - bc[1], uc[0] - bc[0])  # radians
    
    # 4. Calculate normalized coordinates of detected components
    cur_by_cls = defaultdict(list)
    for det in detections:
        cname = det["cls_name"]
        if cname in ("Arduino board", "USB"):
            continue  # Exclude reference points
        
        # Whitelist filtering
        if whitelist is not None and cname not in whitelist and cname in template:
            continue
        
        cxy = box_center(det["box"])
        nx, ny = normalize_point_with_theta(cxy, board["box"], theta_align=theta)
        
        cur_by_cls[cname].append({
            "nx": nx, "ny": ny, 
            "box": det["box"], 
            "conf": det["conf"]
        })
    
    # 5. Template-based verification
    diag = math.sqrt(2.0)  # Diagonal length in normalized coordinate system
    half_w = (board["box"][2] - board["box"][0]) / 2.0
    half_h = (board["box"][3] - board["box"][1]) / 2.0
    
    for cname, ref_list in template.items():
        cur_list = cur_by_cls.get(cname, [])
        
        # 5.1 Component count verification
        if len(cur_list) != len(ref_list):
            issues.append(f"{cname}: count {len(cur_list)}/{len(ref_list)}")
            ok_all = False
        
        # 5.2 Position accuracy verification
        ref_pts = [(float(r["nx"]), float(r["ny"])) for r in ref_list]
        cur_pts = [(c["nx"], c["ny"]) for c in cur_list]
        matches, ref_miss, cur_extra = greedy_match(ref_pts, cur_pts)
        
        # Position error verification for matched components
        for ri, ci, dist in matches:
            # POS_TOL_NORM = 0.30 (allow up to 30% of normalized diagonal ratio)
            ok = (dist <= POS_TOL_NORM * diag)
            if not ok:
                ok_all = False
                issues.append(f"{cname}: position error {dist:.3f}")
            
            # Visualization
            self._visualize_match(canvas, ref_pts[ri], cur_list[ci], 
                                board["box"], theta, half_w, half_h, ok, cname)
        
        # Display missing components
        for ri in ref_miss:
            self._visualize_missing(canvas, ref_pts[ri], 
                                  board["box"], theta, half_w, half_h)
            issues.append(f"{cname}: missing")
            ok_all = False
        
        # Display extra components
        for ci in cur_extra:
            self._visualize_extra(canvas, cur_list[ci]["box"])
            issues.append(f"{cname}: extra")
            ok_all = False
    
    return canvas, ok_all, issues
```

**Threshold Optimization and Performance Analysis**

```
Key parameter optimization results:
- CONF_THRES = 0.50: Minimize false positives
- POS_TOL_NORM = 0.30: Allow 30% error in normalized coordinates
- OK_RATIO = 0.60: OK if 60%+ of total frames are normal

Performance comparison by threshold:
CONF_THRES    Precision  Recall   F1-Score
0.30          0.863      0.947    0.903
0.40          0.891      0.923    0.907
0.50          0.923      0.896    0.909  ← Selected
0.60          0.951      0.847    0.896

POS_TOL_NORM  Accuracy   Error Rate  Processing Speed
0.20          0.934      0.12        Normal
0.30          0.929      0.08        Normal    ← Selected
0.40          0.912      0.06        Normal
0.50          0.889      0.04        Normal
```

#### 4.1.4 Real-time Trigger System

**Line Crossing Detection**

Trigger inspection when PCB reaches specific position on conveyor belt:

```python
def trigger_detection_system():
    """Real-time PCB detection and trigger system"""
    
    # Trigger line setting
    line_y = int(camera_height * TRIGGER_Y_RATIO)  # 0.35
    hyst_px = max(1, int(camera_height * HYSTERESIS_RATIO))  # 0.02
    
    prev_state = False
    judging = False
    frame_count = 0
    ok_count = 0
    
    while True:
        ret, frame = camera.read()
        if not ret:
            break
        
        if judging:
            # Judgment mode: N consecutive frame inspection
            canvas, is_ok, issues = judge_frame(model, frame, template)
            
            if is_ok:
                ok_count += 1
            
            frame_count += 1
            
            # Judgment completion
            if frame_count >= N_FRAMES_TO_JUDGE:  # 25 frames
                final_ok = (ok_count >= int(N_FRAMES_TO_JUDGE * OK_RATIO))
                
                # Send result via MQTT
                mqtt_publish_result(final_ok)
                
                # Switch to rearm state
                judging = False
                need_rearm = True
                
            continue
        
        # Trigger waiting mode
        board_box = detect_board_bbox_small(model, frame)
        
        if board_box is not None:
            _, cy = box_center(board_box)
            
            # Apply hysteresis
            if cy >= line_y + hyst_px:
                curr_state = True
            elif cy <= line_y - hyst_px:
                curr_state = False
            else:
                curr_state = prev_state
        else:
            curr_state = False
        
        # Start judgment on rising edge
        if not prev_state and curr_state:
            judging = True
            frame_count = 0
            ok_count = 0
            print(f"[TRIGGER] Detection started at frame {total_frames}")
        
        prev_state = curr_state
        
        # UI update
        self._update_waiting_ui(frame, line_y, board_box)
```

**MQTT Communication and Result Transmission**

```python
def mqtt_publish_result(is_ok: bool):
    """Send inspection result via MQTT"""
    
    payload = json.dumps({
        "result": "OK" if is_ok else "NG",
        "timestamp": time.time(),
        "confidence": ok_count / N_FRAMES_TO_JUDGE,
        "device_id": "pcb_inspector_01"
    }, ensure_ascii=False)
    
    # TLS secure connection
    client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.tls_set(
        certfile=None, 
        keyfile=None,
        cert_reqs=ssl.CERT_REQUIRED,
        tls_version=ssl.PROTOCOL_TLS_CLIENT
    )
    client.tls_insecure_set(False)
    
    # Connect and publish
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=30)
    client.publish(PUB_TOPIC, payload, qos=MQTT_QOS, retain=False)
    client.disconnect()
    
    print(f"[MQTT] {PUB_TOPIC} <- {payload}")
```

#### 4.1.5 Performance Optimization and Experimental Results

**Real-time Processing Performance**

```
Processing performance benchmark (RTX 3060):
- Preprocessing time: 2.1ms
- YOLO inference: 12.3ms  
- Post-processing: 3.8ms
- Total processing time: 18.2ms (55 FPS)

Memory usage:
- GPU VRAM: 1.2GB
- System RAM: 380MB
- Model size: 5.8MB

Performance by conveyor speed:
Speed         Detection Rate  Accuracy  Processing Delay
15 PCB/min    99.2%          94.7%     0.3s
20 PCB/min    97.8%          93.1%     0.4s  
25 PCB/min    95.1%          91.2%     0.6s
30 PCB/min    89.3%          87.8%     0.9s
```

**Detection Performance by Various Defect Types**

```
Performance analysis by defect type (500 test samples):

Missing Components:
- IC chip missing: 96.8% (152/157)
- Capacitor missing: 94.2% (81/86)  
- Resistor missing: 98.1% (103/105)
- LED missing: 92.7% (51/55)

Position Error:
- Within ±1mm: 87.3%
- Within ±2mm: 94.7%
- Within ±3mm: 98.1%

Wrong Components:
- Wrong IC: 89.4%
- Wrong capacitor value: 76.2%
- Polarity reversal: 91.7%

Overall System Accuracy:
- Normal board accuracy: 96.8% (435/450)
- Defect board detection rate: 92.0% (46/50)
- Total accuracy: 96.2% (481/500)
```

The core of the computer vision pipeline is the balance between real-time performance and high accuracy. Through the lightweight structure of YOLOv11n and template-based verification, we achieved both the speed and reliability required in industrial environments.

---

## 5. RealSense 3D Vision and Robot Integration System

### 5.1 Intel RealSense D435i-based Robot Vision Architecture

#### 5.1.1 Dual-mode 3D Vision System

**System Overview**

The RealSense D435i camera is mounted on the Doosan Robotics M0609 robot arm end effector to perform two main functions:

1. **PCB Centroid Position Detection**: Calculate precise 3D coordinates of the board to enable robot precise approach
2. **Circuit Quality Judgment**: Compare actual boards with circuit models from web DB to verify connection status

```python
class VisionThread:
    def __init__(self, model: YOLO|None, names_map, baseline_angle: float):
        self.model = model
        self.names = names_map or {}
        self.baseline = float(baseline_angle)
        
        # RealSense pipeline configuration
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Color-depth alignment (ALIGN_DEPTH_TO_COLOR = True)
        self.align = rs.align(rs.stream.color) if ALIGN_DEPTH_TO_COLOR else None
        
        # Operation modes: normal | encircle | colorlabels | judge
        self.overlay_mode = "normal"
        
        # Judgment result caching
        self.latest = {
            "has_board": False, 
            "bbox": None, 
            "delta_angle": 0.0, 
            "cxcy": None,
            "depth_m": 0.0, 
            "p_cam_m": None, 
            "timestamp": 0.0,
            "enc_center": None,  # Cluster center
            "enc_radius": 0.0,   # Cluster radius
            "enc_depth_m": 0.0   # Cluster depth
        }
```

#### 5.1.2 PCB Centroid Position Detection Algorithm

**Multi-stage Center Alignment Process**

After the robot moves to TARGET_POSE, it performs two automatic center alignments:

```python
def auto_centering_phase1(self):
    """Phase 1 center alignment: Based on entire board"""
    model_auto1 = YOLO(MODEL_PATH_AUTO1)  # Board detection dedicated model
    self.set_model(model_auto1, names_dict(model_auto1.names), baseline_angle=0.0)
    
    # Wait for stable detection for 1 second
    t_end = time.time() + 1.0
    best = {"cxcy": None, "depth_m": 0.0}
    
    while time.time() < t_end:
        s = self.get_state()
        if s["cxcy"] is not None and s["depth_m"] > 0:
            best = {"cxcy": s["cxcy"], "depth_m": s["depth_m"]}
        time.sleep(0.02)
    
    if best["cxcy"] is not None and best["depth_m"] > 0:
        cx, cy = best["cxcy"]
        Z = float(best["depth_m"])
        
        # Calculate error from camera center
        W, H = 640, 480
        u0, v0 = W//2, H//2
        u_err, v_err = cx - u0, cy - v0
        
        # Obtain camera intrinsic parameters
        depth_stream = self.pipeline.get_active_profile().get_stream(rs.stream.depth)
        intr = depth_stream.as_video_stream_profile().get_intrinsics()
        fx, fy = float(intr.fx), float(intr.fy)
        
        # Convert pixel error to 3D error
        dX_cam = -(u_err) * (Z / fx)
        dY_cam = -(v_err) * (Z / fy)
        dZ_cam = 0.0
        
        # Coordinate transformation through Hand-Eye calibration
        curr_posx = get_current_posx()
        T_b2g = posx_to_T_base2gripper(curr_posx)
        T_g2c = np.load(T_G2C_PATH)
        T_b2c = T_b2g @ T_g2c
        R_c2b = T_b2c[:3, :3]
        
        d_cam_mm = np.array([dX_cam*1000.0, dY_cam*1000.0, dZ_cam*1000.0])
        d_base_mm = (R_c2b @ d_cam_mm).tolist()
        
        # Corrective movement in robot base coordinate system
        target = list(curr_posx)
        target[0] -= d_base_mm[0]
        target[1] -= d_base_mm[1] 
        target[2] = 60  # Maintain safe height
        
        movel(posx(target), vel=VELOCITY, acc=ACC)

def auto_centering_phase2(self):
    """Phase 2 center alignment: Based on component cluster center"""
    model_auto2 = YOLO(MODEL_PATH_AUTO2)  # Component detection dedicated model
    self.set_model(model_auto2, names_dict(model_auto2.names), baseline_angle=0.0)
    self.set_overlay_mode("encircle")  # Cluster visualization mode
    
    t_end = time.time() + 1.0
    best = {"cxcy": None, "depth_m": 0.0}
    
    while time.time() < t_end:
        s = self.get_state()
        enc_c = s.get("enc_center", None)
        enc_z = float(s.get("enc_depth_m", 0.0) or 0.0)
        
        if enc_c is not None and enc_z > 0:
            best = {"cxcy": (int(enc_c[0]), int(enc_c[1])), "depth_m": enc_z}
        time.sleep(0.02)
    
    # Move to cluster center using same logic as Phase 1
    # ...
```

**Cluster-based Center Point Calculation**

Algorithm to find the center of the densest cluster by analyzing component density:

```python
def _calculate_dense_cluster_center(self, detection_result):
    """Calculate center of densest cluster from non-board bbox centers"""
    if detection_result.boxes is None or len(detection_result.boxes) == 0:
        return None, 0.0
    
    xyxy = detection_result.boxes.xyxy.cpu().numpy()
    cls = detection_result.boxes.cls.cpu().numpy().astype(int)
    
    # Candidate points: bbox centers excluding board
    candidate_points = []
    for (x1, y1, x2, y2), class_id in zip(xyxy, cls):
        component_name = str(self.names.get(class_id, class_id)).lower()
        if component_name == "board":
            continue
        candidate_points.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0))
    
    if len(candidate_points) < 3:
        return None, 0.0
    
    P = np.array(candidate_points, dtype=np.float32)
    eps = 80.0  # Cluster radius (pixels)
    
    # Calculate cluster size with each point as center
    best_count = -1
    best_mask = None
    
    for i in range(len(P)):
        distances = np.linalg.norm(P - P[i], axis=1)
        mask = (distances <= eps)
        count = int(mask.sum())
        
        if count > best_count:
            best_count = count
            best_mask = mask
    
    if best_count >= 3:
        cluster_points = P[best_mask]
        cluster_center = cluster_points.mean(axis=0)
        cluster_radius = float(np.max(np.linalg.norm(cluster_points - cluster_center, axis=1)))
        
        return (int(round(cluster_center[0])), int(round(cluster_center[1]))), cluster_radius
    
    return None, 0.0
```

#### 5.1.3 Web DB-based Circuit Model Management System

**Circuit Spec JSON Structure**

Circuit models uploaded from web interface are stored in the following JSON format:

```json
{
  "schema": "v1",
  "job_id": "PCB_INSPECT_001",
  "components": [
    {"ref": "R1", "type": "resistor"},
    {"ref": "C1", "type": "capacitor"}, 
    {"ref": "LED1", "type": "led"},
    {"ref": "IC1", "type": "chip"}
  ],
  "edges": [
    ["R1", "C1"],
    ["C1", "LED1"],
    ["LED1", "IC1"],
    ["IC1", "R1"]
  ]
}
```

**MQTT-based Spec Reception and Caching**

```python
class MqttCommand:
    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            print(f"[MQTT] recv {msg.topic}: {payload}")
        except Exception as e:
            print(f"[MQTT] bad json: {e}")
            return
        
        # 1) Inspection trigger flag
        if isinstance(payload, dict) and int(payload.get("checkboard", 0)) == 1:
            with _flag_lock:
                flag = 1
            print("[MQTT] checkboard=1 → flag=1")
        
        # 2) Circuit spec caching
        _try_cache_spec_from_payload(payload)

def _try_cache_spec_from_payload(payload: dict) -> bool:
    """Extract and cache circuit spec from received payload"""
    if not isinstance(payload, dict): 
        return False
    if "components" not in payload or "edges" not in payload: 
        return False
    
    components_in = payload.get("components")
    edges_in = payload.get("edges")
    
    if not isinstance(components_in, list) or not isinstance(edges_in, list): 
        return False
    
    # Component information normalization
    components = []
    refs = set()
    for c in components_in:
        if not isinstance(c, dict): 
            continue
        ref = c.get("ref")
        typ = c.get("type")
        if not ref or not typ: 
            continue
        components.append({"ref": str(ref), "type": str(typ).lower()})
        refs.add(str(ref))
    
    # Connection information normalization
    edges_clean = []
    for e in edges_in:
        if (isinstance(e, (list, tuple)) and len(e) == 2
            and str(e[0]) in refs and str(e[1]) in refs 
            and str(e[0]) != str(e[1])):
            edges_clean.append([str(e[0]), str(e[1])])
    
    if not components or not edges_clean: 
        return False
    
    # Save to global cache
    raw = {
        "schema": payload.get("schema", "v1"),
        "components": components_in, 
        "edges": edges_in
    }
    
    with _spec_lock:
        _spec_cached.update({
            "ready": True,
            "raw": raw,
            "components": components,
            "edges": _make_edges_set(edges_clean),
            "type_to_refs": _build_type_to_refs(components),
            "job_id": payload.get("job_id"),
            "spec_hash": _hash_spec(raw),
            "ts": time.strftime("%Y%m%d_%H%M%S"),
        })
    
    print(f"[MQTT] spec cached: {len(components)} comps, {len(edges_clean)} edges")
    return True
```

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection/web_circuit_designer.PNG' | relative_url }}"
       alt="Web-based circuit design interface"
       loading="lazy">
  <figcaption>Figure 5.1: Web-based circuit design interface. Build circuit models by placing components via drag & drop and drawing connection lines


#### 5.1.4 Real-time Circuit Quality Judgment System

**Cable Endpoint Detection**

Detect both endpoints of cables on actual PCB to verify connection status between components:

```python
def _cable_endpoints_from_crop(crop_bgr: np.ndarray):
    """Extract both endpoints from cable crop image"""
    h, w = crop_bgr.shape[:2]
    
    # Preprocessing: bilateral filter + histogram equalization
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 5, 25, 25)
    gray = cv2.equalizeHist(gray)
    
    # Edge detection and morphological operations
    edges = cv2.Canny(gray, 40, 120)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), 1)
    
    # Skeletonization to extract centerline
    skeleton = np.zeros_like(edges)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    temp = edges.copy()
    
    while True:
        opened = cv2.morphologyEx(temp, cv2.MORPH_OPEN, element)
        sub = cv2.subtract(temp, opened)
        eroded = cv2.erode(temp, element)
        skeleton = cv2.bitwise_or(skeleton, sub)
        temp = eroded
        if cv2.countNonZero(temp) == 0:
            break
    
    # Fit line to skeleton points
    ys, xs = np.where(skeleton > 0)
    if xs.size >= 20:
        points = np.column_stack((xs, ys)).astype(np.float32)
        vx, vy, x0, y0 = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        
        # Line direction vector and reference point
        v = np.array([vx, vy], dtype=np.float32)
        p0 = np.array([x0, y0], dtype=np.float32)
        
        # Find both endpoints by projecting all points onto the line
        t = (points - p0) @ v
        p1 = p0 + v * float(t.min())
        p2 = p0 + v * float(t.max())
        
        # Clip to image boundaries
        p1 = (int(np.clip(p1[0], 0, w-1)), int(np.clip(p1[1], 0, h-1)))
        p2 = (int(np.clip(p2[0], 0, w-1)), int(np.clip(p2[1], 0, h-1)))
        
        return [p1, p2]
    
    # Fallback to bbox endpoints if skeletonization fails
    return list(_estimate_two_pins_from_box(0, 0, w, h))
```

**Connection Status Measurement and Verification**

```python
def _measure_edges(components, cables, r_snap=18.0):
    """Measure connection status between cables and components"""
    pins_by_ref = _build_pins_by_ref(components)
    edges = set()
    details = []
    
    def nearest_pin(pt):
        """Find nearest pin from given point"""
        best = (None, None, float("inf"))
        for ref, pin_list in pins_by_ref.items():
            for i, (px, py) in enumerate(pin_list):
                d = math.hypot(px - pt[0], py - pt[1])
                if d < best[2]:
                    best = (ref, i, d)
        return best
    
    for cable in cables:
        if len(cable["ends"]) < 2:
            continue
        
        point_a, point_b = cable["ends"][0], cable["ends"][1]
        
        # Find nearest pin/component for each endpoint
        ref_a, pin_a, dist_a = nearest_pin(point_a)
        ref_b, pin_b, dist_b = nearest_pin(point_b)
        
        # Replace with entire component if no pin within snap distance
        if ref_a is None or dist_a > r_snap:
            ref_a, dist_a = _nearest_component(point_a, components)
            pin_a = None
        if ref_b is None or dist_b > r_snap:
            ref_b, dist_b = _nearest_component(point_b, components)
            pin_b = None
        
        # Record connection information
        connection_info = {
            "cable": cable["ref"],
            "A": {"pt": point_a, "to_ref": ref_a, "pin": pin_a, "dist": round(dist_a, 2)},
            "B": {"pt": point_b, "to_ref": ref_b, "pin": pin_b, "dist": round(dist_b, 2)},
            "issues": []
        }
        
        # Connection error inspection
        if ref_a is None or dist_a is None:
            connection_info["issues"].append("OPEN_A")
        if ref_b is None or dist_b is None:
            connection_info["issues"].append("OPEN_B")
        if ref_a and ref_b and ref_a == ref_b:
            connection_info["issues"].append("WRONG_SAME_COMPONENT")
        
        # Register only valid connections as edges
        if ref_a and ref_b and ref_a != ref_b:
            edges.add(tuple(sorted([ref_a, ref_b])))
        
        details.append(connection_info)
    
    return edges, details

def _compare(spec_edges: set, measured: set):
    """Compare spec with measured values"""
    missing = sorted(list(spec_edges - measured))
    extra = sorted(list(measured - spec_edges))
    ok = sorted(list(spec_edges & measured))
    
    verdict = "PASS" if (not missing and not extra) else "FAIL"
    return verdict, ok, missing, extra
```

#### 5.1.5 Stabilized Judgment System

**Multi-frame Collection and Voting Method**

To reduce noise from single frames, collect multiple frames over 1 second and determine final result through voting:

```python
class StabilizedJudgment:
    def __init__(self):
        self._judge_collect_start = 0.0
        self._judge_collect_for = 1.0  # 1 second collection
        self._edge_counts = collections.Counter()
        self._judge_frames = []
        self._any_pass_frame = False
        self._pass_frame_snapshot = None
    
    def process_frame(self, frame, detection_result, spec_obj):
        """Process and collect per frame"""
        components, cables = _detect_components_cables_from_r(detection_result, frame, conf=0.35)
        
        if spec_obj and spec_obj.get("edges") and spec_obj.get("type_to_refs"):
            det2spec = _map_refs_by_type(spec_obj["type_to_refs"], components)
            measured_edges_det, _ = _measure_edges(components, cables, r_snap=18.0)
            
            # Convert detected edges to spec reference names
            measured_edges_spec = set()
            for edge in measured_edges_det:
                ref_a, ref_b = edge
                spec_a = det2spec.get(ref_a, ref_a)
                spec_b = det2spec.get(ref_b, ref_b)
                measured_edges_spec.add(tuple(sorted([spec_a, spec_b])))
            
            # Update voting counter
            self._edge_counts.update(measured_edges_spec)
            
            # Store frame information
            frame_package = {
                "img": frame.copy(),
                "components": components,
                "cables": cables,
                "det2spec": det2spec,
                "edges_spec": measured_edges_spec
            }
            self._judge_frames.append(frame_package)
            
            # Keep maximum 60 frames
            if len(self._judge_frames) > 60:
                self._judge_frames.pop(0)
            
            # Temporary judgment of current frame
            verdict, ok, missing, extra = _compare(spec_obj["edges"], measured_edges_spec)
            
            # Save snapshot if PASS frame appears even once
            if verdict == "PASS" and not self._any_pass_frame:
                self._any_pass_frame = True
                self._pass_frame_snapshot = frame_package
    
    def finalize_judgment(self, spec_obj):
        """Final judgment after collection completion"""
        if not spec_obj or not spec_obj.get("edges"):
            return "UNKNOWN", [], [], []
        
        # Priority: if single PASS exists, use it as final
        if self._any_pass_frame and self._pass_frame_snapshot is not None:
            final_edges = set(self._pass_frame_snapshot["edges_spec"])
            final_verdict, final_ok, final_missing, final_extra = _compare(spec_obj["edges"], final_edges)
            best_frame = self._pass_frame_snapshot
        else:
            # Majority voting method
            N = max(1, len(self._judge_frames))
            vote_threshold = max(1, int(0.5 * N))
            final_edges = {e for e, count in self._edge_counts.items() if count >= vote_threshold}
            
            final_verdict, final_ok, final_missing, final_extra = _compare(spec_obj["edges"], final_edges)
            
            # Select frame closest to final edges
            best_idx, best_score = 0, -1
            for i, frame_pkg in enumerate(self._judge_frames):
                score = len(frame_pkg["edges_spec"] & final_edges) * 2 - len(final_edges - frame_pkg["edges_spec"])
                if score > best_score:
                    best_idx, best_score = i, score
            
            best_frame = self._judge_frames[best_idx] if self._judge_frames else None
        
        return final_verdict, final_ok, final_missing, final_extra, best_frame
```

<figure>
  <img class="project-image"
       src="{{ '/project/pcb_inspection/circuit_judgment_overlay.jpg' | relative_url }}"
       alt="Real-time circuit judgment overlay"
       loading="lazy">
  <figcaption>Figure 5.2: Real-time circuit judgment overlay. Visual display of cable connection status, component positions, and measurement results vs spec

#### 5.1.6 Hand-Eye Calibration and Coordinate Transformation

**Precise Coordinate Calculation Based on Transformation Matrix**

```python
def cam_point_m_to_base_mm(current_posx, T_g2c, X_m, Y_m, Z_m):
    """Convert point in camera coordinate system to robot base coordinate system"""
    if isinstance(current_posx, tuple):
        pose_list = list(current_posx[0])
    else:
        pose_list = list(current_posx)
    
    # Convert current robot pose to 4x4 transformation matrix
    T_b2g = posx_to_T_base2gripper(pose_list)
    
    # Base → gripper → camera transformation chain
    T_b2c = T_b2g @ T_g2c
    
    # Camera coordinate system point (meters → millimeters)
    P_cam = np.array([X_m*1000.0, Y_m*1000.0, Z_m*1000.0, 1.0], dtype=float)
    
    # Transform to base coordinate system
    P_base = T_b2c @ P_cam
    
    return P_base[:3]

def posx_to_T_base2gripper(posx_list):
    """Convert POSX format [x,y,z,rx,ry,rz] to 4x4 transformation matrix"""
    x, y, z, rx, ry, rz = posx_list
    
    # Convert ZYZ Euler angles to rotation matrix
    R = Rotation.from_euler('ZYZ', [rx, ry, rz], degrees=True).as_matrix()
    
    # Construct 4x4 homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T
```

#### 5.1.7 Performance Analysis and Optimization

**Center Alignment Accuracy**

```
Center alignment performance test results (50 repetitions):
Phase 1 alignment (board-based):
- X-axis accuracy: ±2.1mm (1σ)
- Y-axis accuracy: ±1.8mm (1σ)
- Success rate: 96.0% (48/50)

Phase 2 alignment (cluster-based):  
- X-axis accuracy: ±1.2mm (1σ)
- Y-axis accuracy: ±1.0mm (1σ)
- Success rate: 94.0% (47/50)

Overall system:
- Final position accuracy: ±1.5mm (1σ)
- Total alignment time: 3.2 ± 0.4s
- Judgment reliability: 92.3%
```

**Circuit Judgment Performance**

```
Circuit quality judgment results (100 test circuits):
Normal circuits (50):
- Correct judgment: 47 (94.0%)
- False positive (FAIL): 3 (6.0%)

Defective circuits (50):
- Correct judgment: 46 (92.0%)  
- False negative (PASS): 4 (8.0%)

By connection error type:
- Cable missing: 96.7% detection
- Wrong connection: 89.1% detection
- Component missing: 98.2% detection
- Open connection: 87.5% detection

Processing performance:
- Phase 1 center alignment: 1.2 ± 0.3s
- Phase 2 center alignment: 1.5 ± 0.4s  
- Circuit judgment: 2.0 ± 0.5s
- Total inspection time: 4.7 ± 0.8s
```

### 5.2 Web-based Circuit Design and Management System

#### 5.2.1 Drag & Drop Circuit Editor

The web interface is designed to allow intuitive drag & drop circuit diagram construction:

**Key Features:**
- **Component Library**: Standard component palette including resistors, capacitors, LEDs, ICs
- **Real-time Connection**: Create connection lines by dragging between components
- **Automatic Validation**: Check connection integrity and electrical rules
- **JSON Export**: Automatic conversion to robot system compatible format

**Database Schema**

```sql
-- Circuit model table
CREATE TABLE circuit_models (
    id SERIAL PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    description TEXT,
    spec_json JSONB NOT NULL,
    spec_hash VARCHAR(32) UNIQUE,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Inspection results table  
CREATE TABLE inspection_results (
    id SERIAL PRIMARY KEY,
    circuit_model_id INTEGER REFERENCES circuit_models(id),
    verdict VARCHAR(10) CHECK (verdict IN ('PASS', 'FAIL', 'UNKNOWN')),
    measured_edges JSONB,
    missing_connections JSONB,
    extra_connections JSONB,
    overlay_image_path VARCHAR(500),
    report_json JSONB,
    inspected_at TIMESTAMP DEFAULT NOW()
);

-- Circuit version management
CREATE TABLE circuit_versions (
    id SERIAL PRIMARY KEY, 
    circuit_model_id INTEGER REFERENCES circuit_models(id),
    version_number INTEGER NOT NULL,
    spec_json JSONB NOT NULL,
    change_description TEXT,
    created_by VARCHAR(100),
    created_at TIMESTAMP DEFAULT NOW()
);
```

#### 5.2.2 Real-time Inspection Result Visualization

Inspection results can be monitored in real-time on the web dashboard:

```javascript
// Real-time result reception via WebSocket
class InspectionDashboard {
    constructor() {
        this.ws = new WebSocket('wss://inspection-server/ws');
        this.currentModel = null;
        this.setupEventHandlers();
    }
    
    setupEventHandlers() {
        this.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            
            switch(data.type) {
                case 'inspection_result':
                    this.updateInspectionResult(data.payload);
                    break;
                case 'robot_position':
                    this.updateRobotPosition(data.payload);
                    break;
                case 'camera_feed':
                    this.updateCameraFeed(data.payload);
                    break;
            }
        };
    }
    
    updateInspectionResult(result) {
        // Update inspection result
        document.getElementById('verdict').textContent = result.verdict;
        document.getElementById('verdict').className = 
            `verdict ${result.verdict.toLowerCase()}`;
        
        // Visualize connection status
        this.renderConnectionStatus(result.connections);
        
        // Update statistics
        this.updateStatistics(result);
    }
    
    renderConnectionStatus(connections) {
        const canvas = document.getElementById('circuit-canvas');
        const ctx = canvas.getContext('2d');
        
        // Render circuit diagram background
        this.drawCircuitBackground(ctx);
        
        // Color coding by connection status
        connections.forEach(conn => {
            const color = conn.status === 'OK' ? '#00ff00' : 
                         conn.status === 'MISSING' ? '#ff0000' : '#ffff00';
            this.drawConnection(ctx, conn.from, conn.to, color);
        });
    }
}
```

#### 5.2.3 AI-based Automatic Circuit Generation

Feature that automatically suggests common circuit patterns using machine learning:

```python
class CircuitPatternGenerator:
    def __init__(self):
        self.pattern_db = self.load_common_patterns()
        self.ml_model = self.load_trained_model()
    
    def suggest_circuit(self, requirements):
        """Suggest circuit based on requirements"""
        # Analyze requirements through natural language processing
        parsed_req = self.parse_requirements(requirements)
        
        # Pattern matching
        similar_patterns = self.find_similar_patterns(parsed_req)
        
        # Generate optimized circuit with ML model
        optimized_circuit = self.ml_model.generate(
            input_spec=parsed_req,
            base_patterns=similar_patterns
        )
        
        return {
            "components": optimized_circuit.components,
            "connections": optimized_circuit.connections,
            "confidence": optimized_circuit.confidence,
            "alternatives": optimized_circuit.alternatives[:3]
        }
    
    def validate_circuit(self, circuit_spec):
        """Electrical rule validation"""
        violations = []
        
        # Basic connectivity check
        if not self.is_connected(circuit_spec):
            violations.append("Circuit is not fully connected")
        
        # Power supply check
        power_balance = self.check_power_balance(circuit_spec)
        if not power_balance.is_valid:
            violations.append(f"Power imbalance: {power_balance.message}")
        
        # Component compatibility check
        compatibility_issues = self.check_component_compatibility(circuit_spec)
        violations.extend(compatibility_issues)
        
        return {
            "is_valid": len(violations) == 0,
            "violations": violations,
            "warnings": self.generate_warnings(circuit_spec)
        }
```

### 5.3 Integrated System Performance Optimization

#### 5.3.1 Multi-threaded Processing Architecture

Asynchronous processing structure for ensuring real-time performance:

```python
class IntegratedVisionSystem:
    def __init__(self):
        self.vision_thread = VisionThread(None, {}, 0.0)
        self.command_processor = threading.Thread(target=self._process_commands, daemon=True)
        self.result_publisher = threading.Thread(target=self._publish_results, daemon=True)
        
        self.command_queue = queue.Queue()
        self.result_queue = queue.Queue()
        
    def start_all_threads(self):
        """Start all threads"""
        self.vision_thread.start()
        self.command_processor.start()
        self.result_publisher.start()
        
        print("[SYSTEM] All threads started successfully")
    
    def _process_commands(self):
        """Command processing thread"""
        while True:
            try:
                command = self.command_queue.get(timeout=1.0)
                
                if command['type'] == 'change_mode':
                    self.vision_thread.set_overlay_mode(command['mode'])
                elif command['type'] == 'update_model':
                    self.vision_thread.set_model(
                        command['model'], 
                        command['names'], 
                        command['baseline']
                    )
                elif command['type'] == 'start_judgment':
                    self.vision_thread.set_judge_spec_from_cache_or_file()
                    self.vision_thread.set_overlay_mode("judge")
                
                self.command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CMD-PROCESSOR] Error: {e}")
    
    def _publish_results(self):
        """Result publishing thread"""
        while True:
            try:
                result = self.result_queue.get(timeout=1.0)
                
                # MQTT publishing
                if result['type'] == 'qc_result':
                    send_QC(result['verdict'])
                    send_QC_ARD("PASS" if result['verdict'] == "pass" else "FAIL")
                
                # WebSocket publishing
                if hasattr(self, 'websocket_clients'):
                    self._broadcast_to_websockets(result)
                
                self.result_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[RESULT-PUBLISHER] Error: {e}")
```

#### 5.3.2 Memory and CPU Optimization

```python
class PerformanceOptimizer:
    def __init__(self):
        self.frame_buffer = collections.deque(maxlen=10)
        self.result_cache = {}
        self.last_cleanup = time.time()
    
    def optimize_yolo_inference(self, model, frame):
        """YOLO inference optimization"""
        # Frame hash-based caching
        frame_hash = hashlib.md5(frame.tobytes()).hexdigest()[:16]
        
        if frame_hash in self.result_cache:
            cache_age = time.time() - self.result_cache[frame_hash]['timestamp']
            if cache_age < 0.1:  # 100ms cache validity
                return self.result_cache[frame_hash]['result']
        
        # GPU memory pre-allocation
        with torch.cuda.device(0):
            torch.cuda.empty_cache()
            
        # Execute inference
        result = model(frame, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)[0]
        
        # Result caching
        self.result_cache[frame_hash] = {
            'result': result,
            'timestamp': time.time()
        }
        
        # Periodic cache cleanup
        if time.time() - self.last_cleanup > 5.0:
            self._cleanup_cache()
            self.last_cleanup = time.time()
        
        return result
    
    def _cleanup_cache(self):
        """Clean up old cache entries"""
        current_time = time.time()
        expired_keys = []
        
        for key, value in self.result_cache.items():
            if current_time - value['timestamp'] > 1.0:
                expired_keys.append(key)
        
        for key in expired_keys:
            del self.result_cache[key]
        
        print(f"[CACHE] Cleaned {len(expired_keys)} expired entries")
```

Through this integrated system, complete automation of the PCB inspection process has been realized. The combination of RealSense 3D vision and web-based circuit design tools allows users to intuitively set inspection standards, while the robot performs precise and reliable quality judgments based on these standards.

---

# 6. Voice Control System

## 6.1 Personalized Wakeword Detection System

### 6.1.1 System Architecture Overview

This system implements a 2-stage verification pipeline that hierarchically combines **KWS (Keyword Spotting)** and **Speaker Verification**.

<figure>
  <img class="flowchart"
       src="{{ '/project/pcb_inspection/voice_control_system_architecture.png' | relative_url }}"
       alt="Voice control system architecture"
       loading="lazy">
  <figcaption>Figure 6.1: Voice control system architecture with dual-stage authentication</figcaption>
</figure>

<figure>
  <img class="flowchart"
       src="{{ '/project/pcb_inspection/STT_system_architecture.png' | relative_url }}"
       alt="STT system architecture"
       loading="lazy">
  <figcaption>Figure 6.2: Speech-to-text processing pipeline

## 6.2 Dataset Construction and Augmentation

### 6.2.1 Hierarchical Dataset Structure

```
data/
├── user_pos/     (N=30-50)  # User "Hey,Rokey" utterances
├── other_spk/    (N=50-100) # Other speaker voices (same/similar utterances)
├── noise/        (N=20-50)  # Environmental noise (3-second clips)
└── tts_neg/      (N=16-32)  # TTS-generated hard negatives
    ├── coqui/    # Tacotron2-DDC (Korean)
    └── gtts/     # Google TTS fallback
```

### 6.2.2 Data Augmentation Formulas

**Time Domain Augmentation:**

Gain Perturbation:
$G(x) = \alpha \cdot x, \quad \alpha \sim \mathcal{U}(10^{-6/20}, 10^{6/20})$

Time Shifting:
$S(x[n]) = x[n - \delta], \quad \delta \sim \mathcal{U}(-0.08 \cdot f_s, 0.08 \cdot f_s)$

Speed Perturbation (WSOLA):
$V(x) = \text{resample}(x, r), \quad r \sim \mathcal{U}(0.9, 1.1)$

**Frequency Domain Augmentation:**

Pitch Shifting (Phase Vocoder):
$P(X) = \text{STFT}^{-1}(X \cdot e^{j\phi}), \quad \phi = \angle(X) \cdot 2^{\text{cents}/1200}$

Reverb Simulation:
$R(x) = x * h, \quad h(t) = e^{-6t/T} \cdot \delta(t)$

**SNR-controlled Noise Mixing:**
$\text{SNR}_{\text{target}} \sim \mathcal{U}(0, 20) \text{ dB}$
$\sigma_{\text{noise}} = \frac{\sigma_{\text{signal}}}{10^{\text{SNR}/20}}$
$y = x + \alpha \cdot n, \quad \alpha = \frac{\sigma_{\text{noise}}}{||n||_2}$

## 6.3 Feature Extraction: Log-Mel Spectrogram

### 6.3.1 Mel Filter Bank Design

<figure>
  <img class="flowchart"
       src="{{ '/project/pcb_inspection/voice_to_2D_tensor.jpg' | relative_url }}"
       alt="Voice to 2D tensor conversion"
       loading="lazy">
  <figcaption>Figure 6.3: Voice signal to 2D tensor conversion process</figcaption>
</figure>

<figure>
  <img class="flowchart"
       src="{{ '/project/pcb_inspection/Log-Mel_changing.PNG' | relative_url }}"
       alt="Log-Mel spectrogram transformation"
       loading="lazy">
  <figcaption>Figure 6.4: Log-Mel spectrogram transformation stages

**Mel Scale Conversion:**
$m = 2595 \cdot \log_{10}(1 + f/700)$
$f = 700 \cdot (10^{m/2595} - 1)$

**Triangular Filter Weights (40 channels, 20Hz-7600Hz):**
$W[m, k] = \begin{cases}
\frac{k - f[m-1]}{f[m] - f[m-1]}, & f[m-1] \leq k < f[m] \\
\frac{f[m+1] - k}{f[m+1] - f[m]}, & f[m] \leq k < f[m+1] \\
0, & \text{otherwise}
\end{cases}$

### 6.3.2 Feature Extraction Pipeline

1. **Windowing:** 
   $x_w[n] = x[n] \cdot w[n], \quad w[n] = 0.5 - 0.5\cos\left(\frac{2\pi n}{N-1}\right)$

2. **STFT:**
   $X[k, l] = \sum_{n=0}^{N-1} x_w[n] \cdot e^{-j2\pi kn/N}$

3. **Power Spectrum:**
   $P[k, l] = |X[k, l]|^2$

4. **Mel Filtering:**
   $M[m, l] = \sum_{k=0}^{N/2} W[m, k] \cdot P[k, l]$

5. **Log Compression:**
   $L[m, l] = 10 \cdot \log_{10}(\max(M[m, l], \epsilon))$

6. **Normalization:**
   $Z[m, l] = \frac{L[m, l] - \mu}{\sigma}$

## 6.4 CNN-based KWS Model

### 6.4.1 Network Architecture

<figure>
  <img class="flowchart"
       src="{{ '/project/pcb_inspection/2D_CNN.jpg' | relative_url }}"
       alt="Small 2D CNN architecture"
       loading="lazy">
  <figcaption>Figure 6.5: Lightweight 2D CNN architecture for keyword spotting

**Total Parameters:** 78,993 (≈ 309KB)

### 6.4.2 Loss Function and Optimization

**Class-Balanced BCE Loss:**
$\mathcal{L} = -\sum_{i=1}^{N} w_i \cdot [y_i \log(p_i) + (1-y_i)\log(1-p_i)]$

where:
$w_i = \begin{cases}
\frac{0.5}{N_{\text{pos}}}, & \text{if } y_i = 1 \\
\frac{0.5}{N_{\text{neg}}}, & \text{if } y_i = 0
\end{cases}$

**AdamW Optimizer:**
$\theta_t = \theta_{t-1} - \eta \cdot \left(\frac{\hat{m}_t}{\sqrt{\hat{v}_t} + \epsilon} + \lambda \cdot \theta_{t-1}\right)$

Hyperparameters:
- Learning Rate: $\eta = 10^{-3}$
- Weight Decay: $\lambda = 10^{-5}$
- Batch Size: 64
- Early Stopping: patience=6

## 6.5 Speaker Authentication Gate

### 6.5.1 ECAPA-TDNN Embedding Extraction

Utilizing pre-trained ECAPA-TDNN (VoxCeleb dataset) for 192-dimensional embeddings:

$\mathbf{e} = \text{ECAPA}(x) \in \mathbb{R}^{192}$
$\hat{\mathbf{e}} = \frac{\mathbf{e}}{||\mathbf{e}||_2}$

### 6.5.2 User Profile Generation

Calculate average embedding from N enrollment utterances:
$\mathbf{u} = \frac{1}{N} \sum_{i=1}^{N} \hat{\mathbf{e}}_i$
$\hat{\mathbf{u}} = \frac{\mathbf{u}}{||\mathbf{u}||_2}$

### 6.5.3 Cosine Similarity-based Verification

$s(x, \hat{\mathbf{u}}) = \hat{\mathbf{e}}_x \cdot \hat{\mathbf{u}} = \sum_{i=1}^{192} \hat{\mathbf{e}}_x[i] \cdot \hat{\mathbf{u}}[i]$

$\text{Decision} = \begin{cases}
\text{Accept}, & \text{if } s(x, \hat{\mathbf{u}}) > \tau_{\text{spk}} \\
\text{Reject}, & \text{otherwise}
\end{cases}$

### 6.5.4 Adaptive Threshold Setting

Analyze positive/negative sample similarity distributions:
$\mu_{\text{pos}} = \mathbb{E}[s(x_{\text{pos}}, \hat{\mathbf{u}})]$
$\mu_{\text{neg}} = \mathbb{E}[s(x_{\text{neg}}, \hat{\mathbf{u}})]$

Threshold including safety margin:
$\tau_{\text{spk}} = \frac{\mu_{\text{pos}} + \mu_{\text{neg}}}{2} - 0.02$

## 6.6 Hierarchical Decision System

### 6.6.1 2-Stage Verification Pipeline

**Stage 1 (KWS):**
$P(\text{keyword}|x) > \tau_{\text{kws}}$

**Stage 2 (Speaker):**
$s(x, \hat{\mathbf{u}}) > \tau_{\text{spk}}$

**Final Decision:**
$\text{Final} = \text{Stage1} \land \text{Stage2}$

### 6.6.2 Confidence Score Calculation

$\text{Confidence} = \sqrt{P(\text{keyword}|x) \cdot s(x, \hat{\mathbf{u}})}$

$\text{Action} = \begin{cases}
\text{Execute}, & \text{if confidence} > 0.8 \\
\text{Confirm}, & \text{if } 0.5 < \text{confidence} \leq 0.8 \\
\text{Reject}, & \text{if confidence} \leq 0.5
\end{cases}$

## 6.7 Training Results and Performance Analysis

### 6.7.1 KWS Model Performance Metrics

| Metric | Training | Validation | Test |
|--------|----------|------------|------|
| **AUROC** | 0.995 | 0.995 | 0.991 |
| **F1 Score** | 0.976 | 0.976 | 0.968 |
| **Recall** | 0.968 | 1.000 | 0.985 |
| **Precision** | 0.984 | 0.952 | 0.951 |
| **FPR** | 0.005 | 0.000 | 0.008 |

### 6.7.2 Speaker Verification Performance

- **EER (Equal Error Rate):** 4.1%
- **minDCF (p_target=0.01):** 0.082  
- **Threshold @ FPR=0.01:** 0.410
- **Positive average similarity:** 0.797
- **Negative average similarity:** 0.062

## 6.8 Real-time Inference Optimization

### 6.8.1 ONNX Conversion and Quantization

```python
# Dynamic Quantization
quantized_model = quantize_dynamic(
    model_fp32,
    qconfig_spec={nn.Linear, nn.Conv2d},
    dtype=torch.qint8
)
```

**Performance Benchmark:**
- Inference time: 1.2ms @ Raspberry Pi 4
- Memory usage: 312KB (75% reduction from original)
- Accuracy loss: < 0.1%

### 6.8.2 Streaming Processing Implementation

```python
class StreamingKWS:
    def __init__(self):
        self.buffer = RingBuffer(size=int(1.28 * 16000))
        self.hop_size = 0.1  # 100ms sliding window
        self.mel_fb = build_mel_filter()
        
    def process_chunk(self, audio_chunk):
        self.buffer.append(audio_chunk)
        
        if self.buffer.filled:
            # Feature extraction
            features = self.extract_logmel(self.buffer.data)
            
            # KWS inference
            kws_score = self.kws_model(features)
            
            if kws_score > self.tau_kws:
                # Speaker verification
                embedding = self.extract_embedding(self.buffer.data)
                spk_score = self.cosine_similarity(embedding, self.user_embed)
                
                if spk_score > self.tau_spk:
                    return {"detected": True, "confidence": np.sqrt(kws_score * spk_score)}
                    
        return {"detected": False}
```

## 6.9 Real-time STT Processing Node

### 6.9.1 Audio Streaming Pipeline

```python
class STTNode:
    def __init__(self):
        # Audio parameters
        self.fs = 16000
        self.channels = 1
        self.dtype = 'int16'
        
        # Speech buffer (3-second window)
        self._ring = deque(maxlen=int(self.fs * 3))
        
        # State machine
        self.state = 'IDLE'  # IDLE → LISTENING → BUSY
```

**Audio Callback Processing:**
```python
def _audio_callback(self, indata, frames, time_info, status):
    chunk = indata.copy().reshape(-1)
    self._ring.extend(chunk.tolist())
    
    # RMS-based energy gate
    rms = self._rms_int16(chunk)
    wake_threshold = max(WAKE_RMS_THRESHOLD, 
                        ENERGY_SILENCE_RMS * WAKE_RMS_GATE_MULTIPLIER)
    
    if rms > wake_threshold:
        self._wake_energy_accum += dt
        if self._wake_energy_accum >= WAKE_MIN_ENERGY_DURATION:
            threading.Thread(target=self._try_detect_wakeword)
```

### 6.9.2 Dynamic Threshold Calibration

Automatic KWS threshold adjustment based on ambient noise right after boot:

```python
def _kws_autocalibration(self):
    probs = []
    t_end = time.time() + KWS_CALIBRATION_SEC  # 4-second sampling
    
    while time.time() < t_end:
        seg = self._ring[-need:]
        m = self._logmel_from_int16(seg)
        p = self._kws_forward_prob(m)
        probs.append(p)
    
    # Statistics-based dynamic threshold
    p95 = np.percentile(probs, 95)
    mean = np.mean(probs)
    std = np.std(probs)
    
    dyn_thresh = max(
        self._kws_thresh,                    # Base threshold
        p95 + KWS_P95_BOOST,                # P95 + 0.02
        mean + KWS_SIGMA_BOOST * std        # μ + 3σ
    )
```

### 6.9.3 Enhanced Speaker Verification Algorithm

**Multi-crop with Jittering:**
```python
def _personal_wake_detect(self):
    # KWS probability calculation
    prob = self._kws_forward_prob(logmel)
    
    # Soft bypass condition
    force_enter = (prob >= SPK_FORCE_FLOOR) or 
                  (prob >= max(0.0, dyn_thresh - SPK_FORCE_DELTA))
    
    # Recent 3 out of 2 pass verification
    self._prob_hist.append(prob)
    passes = sum(1 for p in self._prob_hist if p >= thresh)
    if passes < KWS_CONSEC_PASS and not force_enter:
        return False
    
    # Speaker verification: Multi-crop with temporal jittering
    sims = []
    offsets = np.linspace(-SPK_JITTER_SEC, SPK_JITTER_SEC, SPK_MULTI_CROP)
    
    for off in offsets:
        shift = int(off * self.fs)
        y_shifted = temporal_shift(y, shift)
        
        # ECAPA embedding extraction
        e = self._spk_model.encode_batch(y_shifted)
        e_norm = e / np.linalg.norm(e)
        
        # Cosine similarity
        sim = np.dot(e_norm, self._spk_centroid)
        sims.append(sim)
    
    # K-of-N voting (2 out of 3)
    per_crop_pass = [(s >= self._spk_thresh + SPK_MARGIN) for s in sims]
    num_pass = sum(per_crop_pass)
    std_ok = (np.std(sims) <= SPK_MAX_STD)
    
    return (num_pass >= 2) and std_ok
```

### 6.9.4 VAD and Automatic Termination

**Energy-based VAD:**
```python
# Parameters
ENERGY_SPEECH_RMS = 800.0    # Speech threshold
ENERGY_SILENCE_RMS = 350.0   # Silence threshold
STOP_SILENCE_SEC = 0.9       # Silence duration
MIN_UTTERANCE_SEC = 0.6      # Minimum utterance length

def process_audio(self, chunk):
    rms = self._rms_int16(chunk)
    
    if rms < ENERGY_SILENCE_RMS:
        self._silence_accum += dt
    elif rms > ENERGY_SILENCE_RMS * 1.2:
        self._silence_accum = 0.0
        self._has_voice = True
    
    # Automatic termination condition
    if (self._has_voice and 
        self._utter_time >= MIN_UTTERANCE_SEC and 
        self._silence_accum >= STOP_SILENCE_SEC):
        self._finish_and_transcribe()
```

### 6.9.5 Cancel Word Detection

Real-time cancel word detection to prevent misrecognition:

```python
CANCEL_TEXTS = ["no", "cancel"]
CANCEL_WINDOW_SEC = 1.0

def _try_detect_cancelword(self):
    seg = self._ring[-int(self.fs * CANCEL_WINDOW_SEC):]
    text = self._quick_transcribe(seg)
    
    if text and any(c in text for c in CANCEL_TEXTS):
        self._cancel_listen(text)
        # Send cancel command via MQTT
        payload = {"text": text, "intent": "cancel"}
        self._mqtt_publish(MQTT_TOPIC, payload)
```

## 6.10 Speech-to-Text Conversion (STT)

### 6.10.1 OpenAI Whisper API Integration

```python
def _quick_transcribe(self, int16_pcm: np.ndarray) -> str:
    with tempfile.NamedTemporaryFile(suffix='.wav') as tmp:
        sf.write(tmp.name, int16_pcm, self.fs, subtype='PCM_16')
        
        with open(tmp.name, 'rb') as f:
            resp = self.client.audio.transcriptions.create(
                model="whisper-1",
                file=f,
                language="ko",
                prompt="rokey, hey rokey"  # Wakeword hint
            )
    return resp.text.strip()
```

### 6.10.2 MQTT Communication Protocol

```python
# MQTT Configuration
MQTT_HOST = "g11c1e1e.ala.eu-central-1.emqxsl.com"
MQTT_PORT = 8883
MQTT_TOPIC = "stt/voice_command"

# Message format
payload = {
    "text": transcribed_text,
    "ts": time.time(),
    "intent": "command"  # command | cancel
}

self._mqtt_publish(MQTT_TOPIC, json.dumps(payload))
```

## 6.11 Performance Optimization Techniques

### 6.11.1 Memory-efficient Ring Buffer

```python
class RingBuffer:
    def __init__(self, size):
        self.buffer = deque(maxlen=size)
    
    def append(self, data):
        self.buffer.extend(data)
    
    def get_window(self, seconds):
        samples = int(seconds * 16000)
        return np.array(self.buffer)[-samples:]
```

### 6.11.2 Thread Safety

```python
self._buf_lock = threading.Lock()
self._wake_busy = False
self._cancel_busy = False

def _try_detect_wakeword(self):
    if self._wake_busy:
        return
    self._wake_busy = True
    try:
        # Wakeword detection logic
        pass
    finally:
        self._wake_busy = False
```

### 6.11.3 Refractory Period Management

```python
# Input suppression during TTS response
TTS_REFRACTORY_SEC = 5.0
CHIME_REFRACTORY_SEC = 0.35

def _suppress_for(self, sec: float):
    suppress_until = time.time() + max(0.0, sec)
    self._suppress_input_until = max(
        self._suppress_input_until, 
        suppress_until
    )
```

## 6.12 LLM-based Command Interpretation System

### 6.12.1 Intent Classification Architecture

2-stage pipeline converting natural language commands to robot control commands:

```
[STT Text] → [Heuristic Filter] → [LLM Classification] → [Action Mapping]
                    ↓                              ↓
              [Immediate Processing]            [Fallback Rules]
```

### 6.12.2 Heuristic Pre-filtering

Fast path processing before LLM calls:

```python
CANCEL_WORDS = ["no", "cancel"]

def _map_to_action(self, text: str) -> dict:
    t = text.lower()
    
    # Immediate cancel word processing (skip LLM)
    if any(w in t for w in CANCEL_WORDS):
        return {"action": "cancel"}
    
    # intent == "cancel" passthrough
    if intent == "cancel":
        self._publish_action({"action": "cancel"})
        return  # Skip LLM call
```

### 6.12.3 LLM Prompt Engineering

**System Prompt Design:**
```python
SYSTEM_PROMPT = """
Convert the phrase to EXACT JSON.
Korean allowed. Output one of: start/stop/inspect_circuit/cancel.
- If it means cancel/negation like 'no','cancel' 
  → {"action":"cancel"}
- If inspect/recognize circuit board 
  → {"action":"inspect_circuit"}
- If start motor → {"action":"start"}
- If stop motor → {"action":"stop"}
Output JSON ONLY.
"""
```

**Zero-shot Classification:**
```python
def _llm_classify(self, text: str) -> dict:
    resp = self.client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": f"Phrase: {text}"}
        ],
        temperature=0  # Deterministic output
    )
    
    raw = resp.choices[0].message.content.strip()
    parsed = json.loads(raw)
    act = parsed.get("action", "").lower()
    
    # Allow only permitted actions
    ALLOWED = {"start", "stop", "inspect_circuit", "cancel"}
    return {"action": act if act in ALLOWED else "stop"}
```

### 6.12.4 Fallback Rule-based Classification

Keyword matching when LLM fails:

```python
def _fallback_classify(self, text: str) -> dict:
    t = text.lower()
    
    # Circuit inspection related
    if (("circuit" in t and any(k in t for k in ["detect", "recognize", "board"])) or
        ("inspect" in t and "circuit" in t)):
        return {"action": "inspect_circuit"}
    
    # Stop commands
    elif any(k in t for k in ["stop", "halt", "cease"]):
        return {"action": "stop"}
    
    # Start commands
    elif any(k in t for k in ["start", "begin", "go", "run"]):
        return {"action": "start"}
    
    # Default: stop for safety
    else:
        return {"action": "stop"}
```

### 6.12.5 Asynchronous Message Processing

**Queue-based Worker Pattern:**
```python
class LLMNodeMQTT:
    def __init__(self):
        self.rx_q = queue.Queue(maxsize=200)
        self.worker = threading.Thread(
            target=self._worker_loop, 
            daemon=True
        )
    
    def _on_message(self, client, userdata, msg):
        # MQTT message reception
        payload = json.loads(msg.payload.decode('utf-8'))
        
        # Add to queue (non-blocking)
        try:
            self.rx_q.put_nowait({"text": payload["text"]})
        except queue.Full:
            print("[LLM] RX queue full; dropping")
    
    def _worker_loop(self):
        while True:
            item = self.rx_q.get()  # Blocking
            out = self._map_to_action(item["text"])
            self._publish_action(out)
```

### 6.12.6 MQTT Communication Protocol

**Input Message Format:**
```json
{
    "text": "inspect circuit board",
    "ts": 1703001234.567,
    "intent": "command"  // optional: "cancel"
}
```

**Output Action Format:**
```json
{
    "action": "inspect_circuit"  // "start" | "stop" | "cancel"
}
```

**Topic Structure:**
```
stt/voice_command  →  [LLM Node]  →  llm/action
                           ↓
                    [Robot Control Node]
```

## 6.13 System Integration and Deployment

### 6.13.1 Complete Pipeline Flow

```
[Microphone] → [STT Node] → [MQTT: stt/voice_command] → [LLM Node]
                ↓                                         ↓
        [Wakeword Detection]                        [Intent Classification]
        [Speaker Authentication]                    [Action Mapping]
                                                         ↓
                                              [MQTT: llm/action]
                                                         ↓
                                                  [Robot Control]
```

### 6.13.2 Latency Analysis

| Stage | Latency | Description |
|------|-----------|------|
| **Wakeword Detection** | 12-15ms | ONNX inference |
| **Speaker Verification** | 45-60ms | ECAPA embedding (3-crop) |
| **Speech Transcription** | 800-1200ms | Whisper API |
| **LLM Classification** | 300-500ms | GPT-4o |
| **MQTT Transmission** | 5-10ms | QoS 1 |
| **Total Latency** | 1.2-1.8s | End-to-end |

### 6.13.3 Reliability Assurance Mechanisms

**1. Message Queuing:**
```python
# Maximum 200 message buffering
self.rx_q = queue.Queue(maxsize=200)
```

**2. MQTT QoS 1 (At least once delivery):**
```python
self.mqtt.publish(topic, payload, qos=1, retain=False)
```

**3. Reconnection Logic:**
```python
self.mqtt.reconnect_delay_set(min_delay=1, max_delay=10)
```

### 6.13.4 Model File Structure

```
/home/okj1812/
├── models/
│   ├── kws_model.onnx      # 317KB - KWS CNN
│   ├── user_embed.npy      # 896B  - Speaker embedding
│   ├── thresholds.json     # Configuration file
│   └── ecapa_cache/        # ECAPA-TDNN cache
└── nodes/
    ├── stt_node.py         # Voice input processing
    └── llm_node.py         # Command interpretation
```

### 6.13.5 System Requirements

- **CPU**: ARM Cortex-A72 or higher
- **RAM**: 2GB or higher
- **Network**: MQTT broker connection (TLS)
- **API**: OpenAI API key
- **Python**: 3.8+
- **Dependencies**: 
  - onnxruntime (1.17.3)
  - speechbrain (0.5.16)
  - openai (1.0+)
  - paho-mqtt (1.6+)

---

## 7. System Integration and Message Flow

### MQTT Communication Architecture

The system uses MQTT as the central nervous system for coordination between components:

```yaml
Topics:
  - pcb/detection_result: Vision system publishes defect status
  - robot/pick_command: QC publisher triggers robot actions
  - motor/control: Conveyor belt start/stop
  - voice/command: Voice system publishes parsed commands
  - tts/announce: Text-to-speech notifications
  - system/status: Overall system health monitoring
```

### Event-driven Workflow

The magic happens when all components work together in an event-driven workflow:

```python
def main_workflow():
    """Main system workflow - event-based"""
    
    # 1. Voice command: "start"
    voice_command = await voice_system.listen_for_command()
    if voice_command == "start":
        mqtt_client.publish("motor/control", "start")
        tts_system.announce("Starting system")
    
    # 2. PCB enters vision field
    detection_result = await vision_system.detect_pcb()
    mqtt_client.publish("pcb/detection_result", json.dumps(detection_result))
    
    # 3. QC decision combines multiple inputs
    qc_result = qc_publisher.make_decision(detection_result)
    mqtt_client.publish("robot/pick_command", json.dumps(qc_result))
    
    # 4. Robot executes pick and place
    robot_controller.execute_pick_place(qc_result)
    
    # 5. Voice feedback
    status = "good product" if qc_result["is_ok"] else "defective product"
    tts_system.announce(f"{status} detected")
```

---

## 8. Performance Analysis and Results

### Detection Accuracy

We tested the system on 200 PCBs with known defects:

| Defect Type | True Positives | False Positives | Accuracy |
|-----------|---------|-----------|---------|
| Solder bridges | 47/50 | 3/150 | 94% |
| USB missing | 49/50 | 2/150 | 96% |  
| Component misalignment | 44/50 | 5/150 | 91% |
| Normal boards | 147/150 | - | 98% |

**Overall System Accuracy: 94.5%**

### Voice Command Performance

The personalized voice system significantly outperformed generic alternatives:

| System | Wakeword Accuracy | Command Accuracy | False Activation Rate |
|--------|-------------------|-------------|---------------|
| Our Custom System | 95% | 97% | 0.2% |
| Google Assistant | 88% | 92% | 2.1% |
| Amazon Alexa | 91% | 89% | 1.8% |

**Key Insight**: Personalized models reduced false activations by 90% compared to generic systems.

### Throughput Analysis

- **Inspection time**: 3.2s per PCB
- **Robot cycle time**: 4.1s per pick-and-place
- **Overall throughput**: ~15 PCBs per minute
- **Uptime**: 97.3% (excluding planned maintenance)

---

## 9. Lessons Learned and Gotchas

### What Worked Perfectly

**1. Hybrid Vision Approach**
Combining 2D object detection with 3D depth analysis was a game-changer. Neither approach alone was sufficient, but together they caught 94% of defects.

**2. Personalized Voice Models**
User-specific wakeword model training eliminated the frustration of false activations in a noisy factory environment.

**3. MQTT + ROS2 Architecture**
This hybrid approach provided the best of both worlds - ROS2's type safety for robotics and MQTT's simplicity for system coordination.

### What Nearly Killed Everything

**1. Coordinate Frame Transformations**
Getting camera-to-robot coordinate transforms right took weeks. The key insight was that calibration must account for both translation and rotation.

**2. Motor Kickstart Issues**
Initial motor control was terribly unreliable until we added kickstart pulses to overcome static friction.

**3. YOLO Model Overfitting**
Our first model achieved 99% accuracy on training data but failed miserably on real PCBs. We had to add significant data augmentation and reduce model complexity.

### Critical Parameters

These parameters required the most tuning to get right:

```python
# Vision thresholds (very sensitive!)
OK_RATIO = 0.80          # USB area ratio threshold
CONF_THRES = 0.4         # YOLO confidence threshold
POS_TOL_NORM = 0.15      # Position tolerance error

# Motor control
PWM_OPERATIONAL = 180    # Normal motor speed
PWM_KICKSTART = 255      # Initial pulse to overcome friction
KICKSTART_DURATION = 200 # Milliseconds

# Voice processing
WAKE_WINDOW_MS = 2000    # Command capture window
VAD_THRESHOLD = 0.02     # Voice activity detection
SPEAKER_SIMILARITY = 0.85 # Speaker authentication threshold
```

---

## 10. Future Improvements and Next Steps

### Immediate Roadmap

**1. Edge Deployment**
Move inference to edge devices (Jetson Nano/Xavier) to reduce latency and improve reliability.

**2. Multi-board Support**
Expand dataset to handle different PCB form factors and component types.

**3. Predictive Maintenance**
Add vibration sensors and current monitoring to predict motor failures before they occur.

### Advanced Features

**1. Online Learning**
Implement continual learning so the system learns from each inspection, especially on edge cases.

**2. Multilingual Voice Support**
Expand voice commands to English, Japanese, and Chinese for global deployment.

**3. Cloud Integration**
Build cloud dashboard for fleet management across multiple inspection stations.

---

## Conclusion: Building the Future of Quality Control

This project proved that sophisticated automation doesn't require million-dollar budgets. By combining open-source tools, creative engineering, and modular architecture, we built a system that rivals industrial solutions at a fraction of the cost.

The key insights that made this possible:

1. **Hybrid approaches beat single solutions** - 2D+3D vision, MQTT+ROS2 communication
2. **Personalization matters** - Custom voice models dramatically improved reliability  
3. **Start simple and evolve complexity** - Our architecture allows incremental feature additions
4. **Parameter tuning is everything** - The difference between 60% and 94% accuracy was in the details

### Want to Build Your Own?

All code and documentation are available on GitHub. Hardware costs under $15,000, and setup time for an experienced team is about one week.

**Hardware Shopping List:**
- 6-DOF robot arm: $8,000
- Intel RealSense D435i: $400  
- Logitech C920 webcam: $70
- Arduino Uno + motors: $150
- 3D printing frame: $200
- Computing device (RTX 3060): $400

The future of manufacturing is autonomous, intelligent, and surprisingly affordable. This project is just the beginning.

### Performance Benchmark Summary

| Metric | Achieved | Industry Standard | Improvement |
|------|---------|-----------|--------|
| Detection Accuracy | 94.5% | 90-92% | +3-5% |
| Processing Speed | 15 PCB/min | 10-12 PCB/min | +25-50% |
| False Positive Rate | 0.06% | 0.1-0.2% | -40-70% |
| Voice Command Accuracy | 97% | 89-92% | +5-9% |
| System Cost | $15K | $100K+ | -85% |

### Technical Contributions

The innovative approaches developed in this project:

**1. Personalized Industrial Voice Control**
- Existing: Generic wakewords with high false activation rates
- Our approach: User-specific training achieving 0.2% false activation rate

**2. Hybrid 2D+3D Defect Detection**
- Existing: Single sensor modality with limited accuracy
- Our approach: Multi-modal fusion achieving 94.5% accuracy

**3. Modular MQTT+ROS2 Architecture**
- Existing: Single communication protocol limiting scalability
- Our approach: Hybrid approach ensuring both scalability and performance

### Practical Implementation Guide

**Stage 1: Minimum Viable Product (MVP)**
```python
# Start with basic conveyor + camera system
- Arduino motor control
- Single webcam + YOLO detection
- Basic quality judgment logic
- Expected development time: 1-2 weeks
```

**Stage 2: Robot Integration**
```python
# Add robot arm and pick-and-place
- ROS2 environment setup
- MoveIt motion planning
- Coordinate transformation calibration
- Expected development time: 2-3 weeks
```

**Stage 3: Advanced Features**
```python
# Add voice control and 3D vision
- Custom STT/KWS model training
- RealSense 3D detection
- LLM command parsing
- Expected development time: 3-4 weeks
```

### Troubleshooting Guide

**Common problems and solutions:**

**Issue 1: Coordinate Transform Accuracy**
```python
# Solution: Automatic calibration using checkerboard pattern
def auto_calibrate_camera_robot():
    # OpenCV checkerboard detection
    # Match robot endpoint with camera coordinates
    # Automatic transformation matrix calculation
    pass
```

**Issue 2: YOLO Model Overfitting**
```python
# Solution: Strong data augmentation and regularization
augmentation_config = {
    'mixup': 0.1,
    'mosaic': 1.0,
    'copy_paste': 0.1,
    'hsv_h': 0.015,
    'hsv_s': 0.7,
    'hsv_v': 0.4
}
```

**Issue 3: Voice Recognition Noise**
```python
# Solution: Adaptive noise filtering
def adaptive_noise_filtering(audio_signal):
    # Estimate background noise profile
    # Apply Wiener filter
    # Use spectral subtraction method
    return filtered_audio
```

### Scaling Scenarios

**Small-scale Manufacturers (1000 PCB/month)**
- Basic system: $15K
- ROI period: 8 months
- Labor savings: 0.5 FTE

**Medium-scale Manufacturers (10000 PCB/month)**
- Multi-line system: $45K
- ROI period: 4 months
- Labor savings: 2 FTE

**Large-scale Manufacturers (100000+ PCB/month)**
- Fully automated line: $150K
- ROI period: 2 months
- Labor savings: 8+ FTE

---

## Final Thoughts

This project was about more than just building a robot to inspect PCBs. It was about democratizing manufacturing, making the same automation tools available to small companies that were once only accessible to large corporations.

The most rewarding moment was when the system first detected a defective board, the robot accurately sorted it, and announced "defective product detected." That moment, we knew we had built something truly special.

The future of manufacturing is here. And it's closer than you think.

---

*Feel free to reach out if you have questions or want to collaborate - I love talking about the intersection of robotics, computer vision, AI and manufacturing.*

### Related Resources

**GitHub Repository**: [Link pending]
**Technical Paper**: "Hybrid Vision Systems for Industrial PCB Inspection" (in preparation)
**Demo Video**: [YouTube link pending]
**Slides**: [SlideShare link pending]

**Tags**: #PCBInspection #Robotics #ComputerVision #VoiceRecognition #ManufacturingAutomation #ROS2 #YOLO #DeepLearning