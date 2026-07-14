# Vehicle-Level Control Architecture for 3-Wheeler Electric Auto-Rickshaw

A comprehensive vehicle-level control unit (VCU) implementation designed for three-wheelers featuring independent rear hub motors. This repository contains the control architecture, mathematical models, and stabilization strategies required to electronically replicate differential behavior and ensure dynamic yaw stability.

## Table of Contents
- [Introduction](#introduction)
- [System Overview](#system-overview)
- [Control Architecture](#control-architecture)
- [Key Control Modules](#key-control-modules)
  - [1. Reference Yaw Model & Stability Control](#1-reference-yaw-model--stability-control)
  - [2. Coordinated Torque Allocation](#2-coordinated-torque-allocation)
  - [3. Traction Control & Slip Estimation](#3-traction-control--slip-estimation)
- [Operational Scenarios](#operational-scenarios)
- [Simulink Model Structure](#simulink-model-structure)
- [Team & Credits](#team--credits)

## Introduction
Electric three-wheelers are increasingly adopting independent wheel hub motors due to superior packaging flexibility, reduced mechanical complexity, and higher powertrain efficiency. However, removing the traditional mechanical differential eliminates passive torque equalization between the driving wheels. Without an electronic vehicle-level controller, asymmetric torque generation can lead to severe instability, such as:
- Oversteer or understeer characteristics
- Torque steer effects
- Yaw divergence during sudden transient maneuvers

This project introduces a VCU architecture that coordinates torque allocation between the left and right rear hub motors to actively stabilize the vehicle and electronically simulate a differential.

## System Overview
The control framework relies on explicit real-time feedback from vehicle sensors to compute exact motor outputs:
- **Sensors (Inputs):**
  - **Inertial Measurement Unit (IMU):** Captures vehicle yaw rate, roll/pitch angles, and lateral/longitudinal accelerations.
  - **Steering Angle Sensor:** Measures the driver's steering wheel/handle input to determine directional intent.
  - **Encoders:** Mounted on the left and right motors to monitor independent wheel speed revolutions.
- **Actuators (Outputs):**
  - **Left Motor Torque ($T_L$)**
  - **Right Motor Torque ($T_R$)**

## Control Architecture
The global control system flows through the following pipeline:
1. **Sensors & State Estimation:** Filters noisy inputs to estimate continuous vehicle longitudinal velocity, true yaw rate, and instantaneous slip.
2. **Driver Demand Generation:** Evaluates throttle position to determine the aggregate propulsion torque requirement ($T_{total}$).
3. **Reference Yaw Model:** Uses vehicle kinematics to calculate the target yaw rate based on steering angle and forward velocity.
4. **Yaw Stability Controller:** Computes the deviation between the target and actual yaw rates (Yaw Error) and applies a feedback loop (PID) to output a corrective yaw moment ($M_{des}$).
5. **Torque Allocation & Safety Layer:** Distributes $T_{total}$ and $M_{des}$ into independent wheel commands while applying tire adhesion limits.

## Key Control Modules

### 1. Reference Yaw Model & Stability Control
Driver intent is modeled kinematically. The desired yaw rate ($\gamma_{ref}$) is determined using:
$$\gamma_{ref} = \frac{V \cdot \tan(\delta)}{L}$$
Where $L$ is the wheelbase, $\delta$ is the steering angle, and $V$ is longitudinal velocity. 

The yaw controller calculates error $\gamma_{error} = \gamma_{ref} - \gamma_{actual}$. A PID controller generates the target stabilizing moment $M_{des}$ to counteract understeer or oversteer:
- **Understeer:** Actual yaw is less than intended. The controller increases the outer wheel torque to assist turning.
- **Oversteer:** Actual yaw exceeds intent. The controller reduces the torque difference or aggregate torque to bring the vehicle back into alignment.

### 2. Coordinated Torque Allocation
The VCU maps the total required force and stabilizing moment simultaneously using the following structural layout:
1. $T_L + T_R = T_{total}$ (Propulsion Constraint)
2. $(T_R - T_L) \cdot \frac{t}{2 \cdot r_w} = M_{des}$ (Yaw Constraint)

Solving these relations yields the explicit torque allocation law:
$$T_L = \frac{T_{total}}{2} - \frac{M_{des} \cdot r_w}{t}$$
$$T_R = \frac{T_{total}}{2} + \frac{M_{des} \cdot r_w}{t}$$
Where $t$ is the rear track width and $r_w$ is the wheel radius. The commands are bounded in real-time by lateral load transfer limits across the rear axle.

### 3. Traction Control & Slip Estimation
To prevent wheel spin-out under split-mu ($\mu$) surfaces or severe acceleration, a tracking filter computes individual wheel slip ratios:
$$\lambda_i = \frac{\omega_i \cdot r_w - V}{V}$$
If $\lambda_i > 0.2$ (20% slip threshold), the safety loop overrides the allocation layer to attenuate the specific motor torque command, forcing the tire back into its optimal adhesion zone.

## Operational Scenarios

### Straight Driving (Longitudinal Motion)
When $\delta = 0$, the desired yaw rate drops to zero. Consequently, $M_{des} = 0$, making $\Delta T = 0$. Both motors receive exactly half of the driver's throttle demand ($T_L = T_R = \frac{T_{total}}{2}$), perfectly mimicking a mechanical differential in a straight line.

### Cornering Driving (Turning Motion)
During high-speed cornering, lateral acceleration shifts the normal weight distribution to the outer wheel. The controller dynamically shifts torque to the outer motor, generating a proactive yaw moment to help carve the corner smoothly while checking the inner wheel for slip.

## Simulink Model Structure
The project is implemented via a multi-layered Simulink architecture:
- `Input Subsystem`: Manages driver controls (Throttle, Steering) and environmental conditions (Friction coefficient $\mu$).
- `Yaw_Ref Subsystem`: Formulates the kinematic target trajectory.
- `Yaw_Controller`: Houses the discrete PID stabilization algorithms.
- `Torque_Allocation`: Computes equations, incorporates load transfer limits, and runs the slip-mitigation loops.
- `Vehicle_plant`: Evaluates the 3-DOF longitudinal, lateral, and yaw dynamics.

## Team & Credits
Developed by **Team Electro**:
- Hirdesh Meena
- Manjeet Marandi
- Abhilash Panda
