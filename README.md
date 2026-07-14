# Vehicle-Level Control Architecture for 3-Wheeler Electric Auto-Rickshaw

A comprehensive vehicle-level control unit (VCU) implementation designed for three-wheelers featuring independent rear hub motors[cite: 1]. This repository contains the control architecture, mathematical models, and stabilization strategies required to electronically replicate differential behavior and ensure dynamic yaw stability[cite: 1].

## Table of Contents
- [Introduction](#introduction)
- [System Overview](#system-overview)
- [Control Architecture](#control-architecture)
- [Simulink Model Architecture](#simulink-model-structure)
- [Key Control Modules](#key-control-modules)
  - [1. Reference Yaw Model & Stability Control](#1-reference-yaw-model--stability-control)
  - [2. Coordinated Torque Allocation](#2-coordinated-torque-allocation)
  - [3. Traction Control & Slip Estimation](#3-traction-control--slip-estimation)
- [Operational Scenarios](#operational-scenarios)
- [Team & Credits](#team--credits)

---

## Introduction
Electric three-wheelers are increasingly adopting independent wheel hub motors due to superior packaging flexibility, reduced mechanical complexity, and higher powertrain efficiency[cite: 1]. However, removing the traditional mechanical differential eliminates passive torque equalization between the driving wheels[cite: 1]. Without an electronic vehicle-level controller, asymmetric torque generation can lead to severe instability, such as:
- Oversteer or understeer characteristics[cite: 1]
- Torque steer effects[cite: 1]
- Yaw divergence during sudden transient maneuvers[cite: 1]

This project introduces a VCU architecture that coordinates torque allocation between the left and right rear hub motors to actively stabilize the vehicle and electronically simulate a differential[cite: 1].

---

## System Overview
The control framework relies on explicit real-time feedback from vehicle sensors to compute exact motor outputs[cite: 1]:
- **Sensors (Inputs):**
  - **Inertial Measurement Unit (IMU):** Captures vehicle yaw rate, roll/pitch angles, and lateral/longitudinal accelerations[cite: 1].
  - **Steering Angle Sensor:** Measures the driver's steering wheel/handle input to determine directional intent[cite: 1].
  - **Encoders:** Mounted on the left and right motors to monitor independent wheel speed revolutions[cite: 1].
- **Actuators (Outputs):**
  - **Left Motor Torque ($T_L$)**[cite: 1]
  - **Right Motor Torque ($T_R$)**[cite: 1]

---

## Control Architecture
The global control system flows through the following pipeline:
1. **Sensors & State Estimation:** Filters noisy inputs to estimate continuous vehicle longitudinal velocity, true yaw rate, and instantaneous slip[cite: 1].
2. **Driver Demand Generation:** Evaluates throttle position to determine the aggregate propulsion torque requirement ($T_{total}$)[cite: 1].
3. **Reference Yaw Model:** Uses vehicle kinematics to calculate the target yaw rate based on steering angle and forward velocity[cite: 1].
4. **Yaw Stability Controller:** Computes the deviation between the target and actual yaw rates (Yaw Error) and applies a feedback loop (PID) to output a corrective yaw moment ($M_{des}$)[cite: 1].
5. **Torque Allocation & Safety Layer:** Distributes $T_{total}$ and $M_{des}$ into independent wheel commands while applying tire adhesion limits[cite: 1].

---

## Simulink Model Architecture

The entire vehicle control unit is implemented in Simulink, organized around a structured feedback loop across dedicated subsystems[cite: 1]:

| Subsystem | Inputs | Outputs | Description |
| :--- | :--- | :--- | :--- |
| **Input** | User-defined parameters | `Thr_out`, `Steer_out`, `Mu_out_R`, `m`, `Mu_out_L` | Generates driver inputs (throttle, steering), vehicle mass ($m$), and road friction coefficients ($\mu_L, \mu_R$) for both tracks[cite: 1]. |
| **Driver_demand** | `Throttle` | `T_total` | Maps the percentage throttle input to the total requested propulsion torque ($T_{total}$)[cite: 1]. |
| **Yaw_Ref** | `v`, `steer_out` | `r_ref` | Generates the kinematic reference yaw rate ($r_{ref}$) based on current velocity and steering angle[cite: 1]. |
| **State_est** | `v`, `r`, `w_L`, `w_R`, `m` | `Fz_R`, `Fz_L`, `Slip_L`, `Slip_R` | Estimates real-time normal tire loads ($F_{z,L}, F_{z,R}$) and longitudinal slip ratios ($Slip_L, Slip_R$) for both rear wheels[cite: 1]. |
| **Torque_Allocation** | `M_des`, `T_total`, `Fz_R`, `Fz_L`, `mu_R`, `Slip_L`, `Slip_R`, `mu_L` | `T_R_out`, `T_L_out` | Computes individual motor torque targets, scales them according to dynamic weight transfer, and applies slip-based traction limiting[cite: 1]. |
| **Yaw_Controller** | `r`, `r_ref` | `M_des` | Houses the discrete-time PID feedback loop to regulate the yaw rate error and output the stabilizing yaw moment ($M_{des}$)[cite: 1]. |
| **Vehicle_plant** | `T_L`, `T_R`, `T_L1`, `T_R1`, `v_i` (feedback), `theta` (slope), `m` | `v`, `r`, `w_L`, `w_R` | Evaluates physical vehicle dynamics to output velocity ($v$), actual yaw rate ($r$), and wheel angular velocities ($\omega_L, \omega_R$)[cite: 1]. |

### Real-Time Visualization Scopes
- **`v`**: Monitors vehicle forward speed profile[cite: 1].
- **`r and r_ref`**: Superimposes actual yaw rate ($r$) over target yaw rate ($r_{ref}$) to evaluate trajectory tracking[cite: 1].
- **`T_L and T_R`**: Tracks left and right motor torque commands to confirm active electronic differential behavior[cite: 1].
- **`Slip_L and Slip_R`**: Monitors wheel slip metrics to verify traction control intervention[cite: 1].

---

## Key Control Modules

### 1. Reference Yaw Model & Stability Control
Driver intent is modeled kinematically. The desired yaw rate ($r_{ref}$) is determined using[cite: 1]:
$$r_{ref} = \frac{v \cdot \tan(\delta)}{L}$$
Where $L$ is the wheelbase, $\delta$ is the steering angle, and $v$ is longitudinal velocity[cite: 1]. 

The yaw controller calculates error $e_r = r_{ref} - r_{actual}$[cite: 1]. A PID controller generates the target stabilizing moment $M_{des}$ to counteract understeer or oversteer[cite: 1]:
- **Understeer:** Actual yaw is less than intended. The controller increases outer wheel torque to assist turning[cite: 1].
- **Oversteer:** Actual yaw exceeds intent. The controller reduces the torque difference or aggregate torque to bring the vehicle back into alignment[cite: 1].

### 2. Coordinated Torque Allocation
The VCU maps the total required force and stabilizing moment simultaneously[cite: 1]:
1. $T_L + T_R = T_{total}$ (Propulsion Constraint)[cite: 1]
2. $(T_R - T_L) \cdot \frac{t}{2 \cdot r_w} = M_{des}$ (Yaw Constraint)[cite: 1]

Solving these relations yields the explicit torque allocation law[cite: 1]:
$$T_L = \frac{T_{total}}{2} - \frac{M_{des} \cdot r_w}{t}$$
$$T_R = \frac{T_{total}}{2} + \frac{M_{des} \cdot r_w}{t}$$
Where $t$ is the rear track width and $r_w$ is the wheel radius[cite: 1]. Commands are bounded in real-time by lateral load transfer limits across the rear axle[cite: 1].

### 3. Traction Control & Slip Estimation
To prevent wheel spin-out under split-mu ($\mu$) surfaces or severe acceleration, a tracking filter computes individual wheel slip ratios[cite: 1]:
$$\lambda_i = \frac{\omega_i \cdot r_w - v}{v}$$
If $\lambda_i > 0.2$ (20% slip threshold), the safety loop overrides the allocation layer to attenuate the specific motor torque command, forcing the tire back into its optimal adhesion zone[cite: 1].

---

## Operational Scenarios

### Straight Driving (Longitudinal Motion)
When $\delta = 0$, the desired yaw rate drops to zero[cite: 1]. Consequently, $M_{des} = 0$, making $\Delta T = 0$[cite: 1]. Both motors receive exactly half of the driver's throttle demand ($T_L = T_R = \frac{T_{total}}{2}$), perfectly mimicking a mechanical differential in a straight line[cite: 1].

### Cornering Driving (Turning Motion)
During high-speed cornering, lateral acceleration shifts the normal weight distribution to the outer wheel[cite: 1]. The controller dynamically shifts torque to the outer motor, generating a proactive yaw moment to help carve the corner smoothly while checking the inner wheel for slip[cite: 1].

---

## Team & Credits
Developed by **Team Electro**:
- Hirdesh Meena[cite: 1]
- Manjeet Marandi[cite: 1]
- Abhilash Panda[cite: 1]
