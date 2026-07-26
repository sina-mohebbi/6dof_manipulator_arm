# 6-DOF Robotic Manipulator — Modbus & CAN Control

![Arduino](https://img.shields.io/badge/platform-Arduino-00979D?logo=arduino&logoColor=white)
![Modbus RTU](https://img.shields.io/badge/fieldbus-Modbus%20RTU-orange)
![CAN Bus](https://img.shields.io/badge/fieldbus-CAN%20Bus-red)
![License](https://img.shields.io/badge/license-GPL--3.0-green)

Control system for a 6 degree-of-freedom robotic arm built around the Modbus RTU and CAN
fieldbus protocols. An Arduino runs the arm as a Modbus slave, driving the DC motors,
stepper, servos and sensors, while a Citect SCADA master on a PC handles supervisory
control and monitoring. The arm can also be driven remotely over Bluetooth from a phone.

This is my B.Sc. project at the School of Electrical & Computer Engineering, University of
Tehran (2023), done in the Industrial Automation & Intelligent Processing Lab. It extends
an earlier lab manipulator by adding the CAN bus link and mobile remote control.

<p align="center">
  <img src="docs/images/gripper-servo.jpg" alt="Servo-driven gripper end-effector, with the 6-DOF arm in the background" width="420">
  <br>
  <em>The servo-actuated gripper (labeled “SERVO2”), with the manipulator arm behind it.</em>
</p>

## Features

- 6-DOF arm driven by 3 DC gear-motors, a stepper, and 2 servos with a gripper end-effector
- Modbus RTU slave on the Arduino Mega, supervised by a Citect SCADA master over RS-485
- CAN bus node-to-node link (Arduino Uno + dual MCP2515)
- Remote control over Bluetooth (HC-05) from the Dabble mobile app
- Manual and Auto operating modes: jog/teach the joints, or run to target angles
- Closed-loop feedback from optical encoders, plus piezo vibration sensing on the arm body
- Hardware diode limit circuit that caps joint travel independently of the software
- Auto-homing to the zero position on power-up using micro-switches and encoders

## Table of Contents
- [System Architecture](#system-architecture)
- [Control Modes](#control-modes)
- [Kinematics](#kinematics)
- [Hardware](#hardware)
- [Bill of Materials](#bill-of-materials)
- [Communication Protocols](#communication-protocols)
- [Pin Mapping](#pin-mapping-arduino-mega)
- [Modbus Register Map](#modbus-register-map)
- [Control Flow & Data Path](#control-flow--data-path)
- [SCADA HMI](#scada-hmi)
- [Wiring](#wiring)
- [Repository Layout](#repository-layout)
- [Building & Flashing](#building--flashing)
- [License](#license)

## System Architecture

The Citect SCADA PC and the mobile app sit at the top. The Arduino Mega runs the
manipulator as a Modbus slave and drives the motors through an L298N driver while reading
the encoders and piezo vibration sensors. A second Arduino Uno links in over CAN using two
MCP2515 transceivers, and an HC-05 module adds the Bluetooth remote link.

![System architecture](docs/images/architecture.png)

```mermaid
flowchart TB
    subgraph Supervisory["Supervisory layer"]
        SCADA["Citect SCADA<br/>(PC master)"]
        APP["Dabble App<br/>(mobile)"]
    end
    subgraph Field["Field / control layer"]
        MEGA["Arduino Mega 2560<br/>Modbus slave"]
        UNO["Arduino Uno<br/>CAN node"]
    end
    subgraph Actuators["Actuators & sensors"]
        L298["L298N driver"]
        MOTORS["3× DC motors · stepper · 2× servos · gripper"]
        SENS["Encoders · piezo vibration · micro-switches"]
    end

    SCADA <-->|Modbus RTU / RS-485| MEGA
    APP <-->|Bluetooth| HC05["HC-05"] <--> UNO
    UNO <-->|CAN bus / MCP2515| MEGA
    MEGA --> L298 --> MOTORS
    SENS --> MEGA
```

## Control Modes

The firmware has two top-level modes, selected over Modbus:

| Mode | Description |
|------|-------------|
| **Local control** | The arm is driven from switches mounted at the robot. |
| **Remote control** | Commanded from Citect SCADA or the mobile app. Has two sub-modes: |
| &nbsp;&nbsp;• *Manual* | Jog each joint with on-screen buttons. Used for calibration, homing and teaching waypoints. |
| &nbsp;&nbsp;• *Auto* | Send target joint angles/positions and the arm moves there on its own. |

On power-up the arm homes to its zero position using the micro-switches and encoders. A
hardware diode circuit enforces the angle limits, so the arm cannot travel past its safe
range even if the software goes wrong. A gripper at the end effector picks and places a
cubic object on an automation line.

```mermaid
stateDiagram-v2
    [*] --> Homing: Power on
    Homing --> Idle: Zeroed (micro-switches + encoders)
    Idle --> Local: Loc_Rem = 0 (panel switches)
    Idle --> Remote: Loc_Rem = 1 (SCADA / app)
    Local --> Idle
    state Remote {
        [*] --> Manual
        Manual --> Auto: waypoints taught
        Auto --> Manual: recalibrate
    }
    Remote --> Idle: stop
```

## Kinematics

The arm has an R‑R‑R‑R‑R‑P configuration: five revolute joints for orientation plus one
prismatic joint before the gripper for linear reach. The forward kinematics use
Denavit–Hartenberg (DH) parameters, where each link is described by four values: joint
angle *θ*, link offset *d*, link length *a*, and link twist *α*. The homogeneous transform
between two consecutive links is:

$$
T_i^{\,i-1} =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Chaining these transforms from base to end-effector gives the gripper pose from the joint
angles read back over Modbus (`Angle1`–`Angle3` and the stepper position).

## Hardware

| Component | Role |
|-----------|------|
| Arduino Mega 2560 | Main controller / Modbus slave, motor & sensor I/O |
| Arduino Uno | Secondary node on the CAN bus |
| L298N V3 | Dual H-bridge driver for the DC motors |
| 3× DC gear-motors | Main / joint-2 / joint-3 rotation (with encoders) |
| Stepper motor | Linear / gripper axis |
| 2× Servo motors | Gripper & wrist |
| Optical encoders | Joint angle feedback (slotted-disk + photo-sensor) |
| Piezo vibration sensors | Detect sudden motion / vibration on the arm body |
| Micro-switches | Homing / end-stop limits |
| MCP2515 (×2) | SPI-to-CAN transceivers |
| HC-05 | Bluetooth link for mobile remote control |

The end-effector is an aluminum parallel-jaw gripper actuated by a servo (the two `servo`
channels are driven from the Mega):

<p align="center">
  <img src="docs/images/gripper-servo-2.jpg" alt="Close-up of the gripper servo wiring into the terminal block" width="360">
  <br>
  <em>Gripper servo wiring into the arm's terminal block.</em>
</p>

## Bill of Materials

| Qty | Part | Notes |
|----:|------|-------|
| 1 | Arduino Mega 2560 | Main controller / Modbus slave |
| 1 | Arduino Uno | CAN node |
| 1 | L298N V3 dual H-bridge | DC motor driver |
| 3 | DC gear-motor + optical encoder | Joints 1–3 |
| 1 | Stepper motor (5-wire) | Linear / gripper axis |
| 2 | Servo motor | Gripper & wrist |
| 3 | Piezo vibration sensor | Arm-body vibration sensing |
| 4+ | Micro-switch | Homing / end-stops |
| 2 | MCP2515 CAN module | SPI ↔ CAN transceiver |
| 1 | HC-05 Bluetooth module | Mobile remote link |
| 1 | RS-485 transceiver (MAX485 / shield) | Modbus physical layer |
| — | Terminals, diodes (safety circuit), wiring | See wiring diagrams |

## Communication Protocols

- **Modbus RTU** over RS-485 links the Arduino slave and the Citect SCADA master
  (9600 baud, slave ID 1). Uses function 3 (read holding registers) and function 16
  (write multiple registers).
- **CAN bus** links the two Arduino nodes via MCP2515.
- **TTL / Serial** and **Bluetooth (HC-05)** provide mobile remote control through the
  Dabble app.
- The wider lab automation line also uses AS-i and Profibus with a PLC (background from the
  thesis; not part of this repo).

| Protocol | Role in this system | Medium | Typical speed | Topology |
|----------|---------------------|--------|---------------|----------|
| **Modbus RTU** | SCADA master ↔ Arduino slave | RS-485 serial | 9600 baud (configured) | Multi-drop, master/slave |
| **CAN** | Arduino ↔ Arduino node link | Differential pair (MCP2515) | up to 1 Mbit/s | Multi-master bus |
| **Bluetooth (HC-05)** | Mobile app ↔ Arduino | 2.4 GHz / TTL UART | 9600–115200 baud | Point-to-point |
| AS-i | Line-level sensors/actuators (lab line) | 2-wire | ~167 kbit/s | Bus |
| Profibus | PLC network (lab line) | RS-485 / fiber | up to 12 Mbit/s | Bus |

## Pin Mapping (Arduino Mega)

Digital pins as defined in the firmware. DC-motor IN/EN pins drive the L298N; encoder pulse
and reset (homing switch) pins are inputs.

| Function | Pin(s) | Direction |
|----------|--------|-----------|
| Main motor — IN1 / IN2 / EN | 0 / 1 / 3 | out |
| Motor 2 — IN1 / IN2 / EN | 2 / 7 / 5 | out |
| Motor 3 — IN1 / IN2 / EN | 14 / 15 / 6 | out |
| Stepper — IN1 / IN2 / IN3 / IN4 | 10 / 11 / 12 / 13 | out |
| Encoder pulse — Main / A2 / A3 | 16 / 17 / 18 | in |
| Homing reset — Main / A2 / A3 | 9 / 19 / 4 | in |
| Stepper homing reset | 8 | in |
| Servo 1 | 23 | out |

> Pin numbers come straight from the `#define` block in the firmware and reflect the
> as-built wiring. The main-motor pins share the Mega's hardware `Serial0` (0/1) used by the
> Modbus link, so move them to free pins if you rewire from scratch.

## Modbus Register Map

The slave uses a single holding-register array (Modbus functions 3 and 16 share it).
Addresses are the zero-based index into that array, so use them directly as the register
addresses in the SCADA tag config. Command registers are written by the master;
status/position registers are read back.

| Addr | Register | Direction | Meaning |
|-----:|----------|-----------|---------|
| 0 | `Main_Motor_CMD` | write | Main motor jog (+/- direction) |
| 1 | `Motor2_CMD` | write | Joint-2 jog |
| 2 | `Motor3_CMD` | write | Joint-3 jog |
| 3 | `StepperMotor_CMD` | write | Stepper jog |
| 4 | `Main_Motor_Pos_CMD` | write | Main motor target angle (auto) |
| 5 | `Motor2_Pos_CMD` | write | Joint-2 target angle (auto) |
| 6 | `Motor3_Pos_CMD` | write | Joint-3 target angle (auto) |
| 7 | `StepperMotor_Pos_CMD` | write | Stepper target position (auto) |
| 8 | `Angle1` | read | Measured main angle |
| 9 | `Angle2` | read | Measured joint-2 angle |
| 10 | `Angle3` | read | Measured joint-3 angle |
| 11 | `Position4` | read | Gripper position |
| 12 | `StepperMotor_Pos` | read | Stepper position |
| 13 | `Man_Auto` | write | 0 = manual, 1 = auto |
| 14 | `Main_Motor_Status` | read | Run = 1 / Stop = 0 |
| 15 | `Motor2_Status` | read | Run = 1 / Stop = 0 |
| 16 | `Motor3_Status` | read | Run = 1 / Stop = 0 |
| 17 | `StepperMotor_Status` | read | Run = 1 / Stop = 0 |
| 18 | `Loc_Rem` | write | 0 = local, 1 = remote |
| 19 | `servo1_CMD` | write | Servo command |
| 20 | `TOTAL_ERRORS` | read | Modbus error counter since start |

Slave config: 9600 baud, slave ID 1, TX-enable on pin 6 (`modbus_configure(9600, 1, 6, …)`).
See [`firmware/Manipulator_970818.ino`](firmware/Manipulator_970818.ino) for the full enum
and control logic.

## Control Flow & Data Path

**Closed-loop joint position control (auto mode).** Each DC joint drives toward its target
angle while the optical encoder increments the measured angle by `step_angle` (6° per
pulse) until the target is reached:

```mermaid
flowchart TD
    A[Read target from Modbus<br/>*_Pos_CMD] --> B{Homing switch<br/>pressed?}
    B -->|Yes| Z[Angle = 0<br/>write to register]
    B -->|No| C{Measured Angle<br/>== target?}
    C -->|Yes| D[Disable motor<br/>Status = 0]
    C -->|No| E[Drive motor toward target<br/>via L298N · Status = 1]
    E --> F{Encoder pulse<br/>edge detected?}
    F -->|Yes| G[Angle += step_angle 6°<br/>write to register]
    F -->|No| E
    G --> C
```

**Command & feedback round-trip.** The SCADA master writes setpoints and polls back the
live angles and status flags over Modbus:

```mermaid
sequenceDiagram
    participant S as Citect SCADA (master)
    participant A as Arduino Mega (slave)
    participant M as Motor + Encoder
    S->>A: FN16 write *_Pos_CMD (target angle)
    A->>M: Drive via L298N toward target
    M-->>A: Encoder pulses (angle feedback)
    A->>A: Update Angle* / *_Status registers
    S->>A: FN3 read Angle*, *_Status
    A-->>S: Current angles + run/stop flags
```

## SCADA HMI

The Citect SCADA project (`scada/robot_arm_11.ctz`) has a Panel Control page with separate
Manual and Auto sections, live joint-angle readouts, per-motor status lamps, and an
animated view of the arm.

![SCADA HMI panel](docs/images/scada-hmi.png)

> `.ctz` is a Citect SCADA project backup — open it with AVEVA/Schneider Electric Citect
> SCADA (Citect Explorer → *Restore*).

## Wiring

**Motors, stepper & servos → L298N driver / terminals → Arduino Mega**

![Motor and servo wiring](docs/images/wiring-motors.png)

**Encoders & piezo vibration sensors → Arduino Mega**

![Sensor wiring](docs/images/wiring-sensors.png)

## Repository Layout

```
.
├── firmware/
│   └── Manipulator_970818.ino   # Arduino Mega firmware (Modbus slave + motor/sensor logic)
├── scada/
│   └── robot_arm_11.ctz         # Citect SCADA supervisory project (HMI master)
├── docs/
│   └── images/                  # Architecture, HMI and wiring diagrams
└── README.md
```

## Building & Flashing

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the required libraries: SimpleModbusSlave and the built-in Stepper library.
3. Open `firmware/Manipulator_970818.ino`, select Arduino Mega 2560, and upload.
4. Wire the RS-485 transceiver, connect the Citect SCADA project as a Modbus master
   (slave ID 1, 9600 baud), and restore `scada/robot_arm_11.ctz`.

## License

The firmware is based on Zihatec's RS422/RS485 shield example and the SimpleModbus library,
and is released under the GNU GPL v3 — see [LICENSE](LICENSE).

## References

- *Development of a Robotic Arm Control System Using CAN Protocol*
- *Optimization of CAN Communication for 6DOF Manipulator Control*

## Author

**Sina Mohebbi** — B.Sc. Electrical & Computer Engineering, University of Tehran.
Industrial Automation & Intelligent Processing Lab, 2023.
