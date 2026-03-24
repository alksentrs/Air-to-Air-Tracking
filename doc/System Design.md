# System Design Document (SDD)

## Airborne Radar Tracking System

---

## 1. Introduction

### 1.1 Purpose

This document describes the design and implementation of a 2D airborne radar tracking system. The system simulates:

* Aircraft motion (circular trajectory)
* Multiple drone targets (constant velocity)
* Radar measurements (range and azimuth)
* Target tracking using Kalman-based estimators

The goal is to provide a modular, extensible MATLAB OOP framework suitable for:

* Research in tracking and estimation
* Extension to EKF/UKF, Particle Filters, and Lie-group filters
* Multi-target tracking scenarios

---

### 1.2 Scope

The system is developed in three stages:

1. Dynamic Simulation (Ground Truth)
2. Radar Measurement Generation
3. Target Tracking (Kalman / EKF)

---

### 1.3 Definitions

| Symbol   | Description             |
| -------- | ----------------------- |
| {W}      | Inertial (world) frame  |
| {B}      | Aircraft body frame     |
| ( p_a )  | Aircraft position       |
| ( p_t )  | Target position         |
| ( R_a )  | Rotation matrix (SO(2)) |
| ( \rho ) | Range                   |
| ( \phi ) | Azimuth                 |

---

## 2. System Overview

The system simulates a radar mounted on a moving aircraft detecting and tracking drones.

### Key Components

* Aircraft Dynamics Model
* Drone Target Models
* Radar Sensor Model
* Tracking Filter (EKF)
* Visualization and Logging

---

### 2.1 High-Level Flow

```
Ground Truth Simulation
        ↓
Relative Geometry Computation
        ↓
Radar Measurement Generation
        ↓
Tracking Filter (EKF)
        ↓
Evaluation & Visualization
```

---

## 3. Mathematical Modeling

### 3.1 Reference Frames

* Inertial frame {W}
* Aircraft body frame {B}

Transformation:

```
p_rel^B = R_a^T (p_t - p_a)
```

---

### 3.2 Aircraft Model

Circular motion:

* Speed: ( v_a = 100 , m/s )
* Radius: ( R = 1000 , m )

Angular rate:

```
ω_a = v_a / R
```

State propagation:

```
θ_a(k+1) = θ_a(k) + ω_a dt

p_a(k+1) = p_a(k) + v_a [cos(θ_a); sin(θ_a)] dt
```

---

### 3.3 Drone Model

Constant Velocity (CV):

```
p_t(k+1) = p_t(k) + v_t dt
v_t = constant
```

Typical speed:

```
||v_t|| ≈ 30 m/s
```

---

### 3.4 Radar Measurement Model

Relative position in body frame:

```
p_rel^B = [x_B; y_B]
```

Measurements:

```
ρ = sqrt(x_B² + y_B²)
φ = atan2(y_B, x_B)
```

Measurement vector:

```
z = [ρ; φ]
```

---

### 3.5 Noise Model

Additive Gaussian noise:

```
ρ_meas = ρ + n_ρ
φ_meas = φ + n_φ
```

Where:

* ( n_ρ \sim \mathcal{N}(0, σ_ρ^2) )
* ( n_φ \sim \mathcal{N}(0, σ_φ^2) )

---

## 4. Software Architecture

### 4.1 Design Principles

* Object-Oriented MATLAB (classdef)
* Separation of concerns
* Modular and extensible design
* Explicit frame handling
* Reusable math utilities

---

### 4.2 Directory Structure

```
/src
  /config
  /models
  /math
  /sensors
  /simulation
  /tracking
  /visualization

/scripts
  run_stage1_simulation.m
  run_stage2_radar_generation.m
  run_stage3_tracking.m
```

---

### 4.3 Core Classes

#### Simulation

* SimulationConfig
* Scenario
* Simulator

#### Models

* AircraftModel
* DroneModel

#### Math

* CoordinateUtils

#### Sensors

* RadarSensor
* RadarMeasurement

#### Tracking

* EKFTracker
* Track
* TrackManager

#### Visualization

* TrajectoryVisualizer
* MeasurementPlotter

---

## 5. Stage 1 — Dynamic Simulation

### 5.1 Objective

Simulate ground truth trajectories of:

* Aircraft (circular motion)
* Multiple drones (linear motion)

---

### 5.2 Features

* Configurable simulation time and step
* Multiple drones
* Real-time visualization

---

### 5.3 Outputs

* Aircraft trajectory
* Drone trajectories
* Heading visualization

---

## 6. Stage 2 — Radar Measurement Generation

### 6.1 Objective

Generate radar detections from aircraft perspective.

---

### 6.2 Processing Steps

1. Compute relative position:

```
p_rel = p_t - p_a
```

2. Transform to body frame:

```
p_rel^B = R_a^T p_rel
```

3. Compute measurements:

```
ρ, φ
```

4. Add noise

---

### 6.3 Outputs

* Range vs time
* Azimuth vs time
* True vs noisy comparison

---

## 7. Stage 3 — Target Tracking (EKF)

### 7.1 State Definition

```
x = [px; py; vx; vy]
```

---

### 7.2 Dynamic Model

```
x(k+1) = F x(k) + w
```

```
F = [1 0 dt 0
     0 1 0 dt
     0 0 1  0
     0 0 0  1]
```

---

### 7.3 Measurement Model

Nonlinear:

```
z = h(x)
```

Depends on:

* Target state
* Aircraft state
* Frame transformation

---

### 7.4 EKF Equations

Prediction:

```
x̂⁻ = F x̂
P⁻ = F P Fᵀ + Q
```

Update:

```
y = z - h(x̂⁻)
S = H P⁻ Hᵀ + R
K = P⁻ Hᵀ S⁻¹

x̂ = x̂⁻ + K y
P = (I - K H) P⁻
```

---

### 7.5 Key Considerations

* Angle normalization (φ)
* Correct Jacobian computation
* Dependency on aircraft motion

---

## 8. Visualization and Metrics

### 8.1 Plots

* True vs estimated trajectory
* Position error
* Range/Azimuth residuals

---

### 8.2 Metrics

* Position RMSE
* Innovation consistency (optional NEES)

---

## 9. Configuration Parameters

* Simulation time
* Sampling time
* Noise parameters (Q, R)
* Number of targets
* Initial conditions

---

## 10. Future Extensions

### 10.1 Tracking Enhancements

* UKF
* Particle Filter
* PF on Lie Groups

---

### 10.2 Modeling Enhancements

* SE(2) / SE(3)
* Aircraft uncertainty
* Maneuvering targets

---

### 10.3 Radar Enhancements

* Clutter
* False alarms
* Detection probability
* Scan patterns

---

### 10.4 Multi-Target Tracking

* Data association (NN, JPDA, MHT)
* Track management

---

## 11. Engineering Considerations

* Numerical stability
* Angle wrapping
* Modularity
* Testability
* Parameter tuning

---

## 12. Conclusion

This system provides a structured, extensible foundation for airborne radar tracking simulation and estimation.

It is designed to evolve toward:

* Advanced filtering (EKF, UKF, PF)
* Lie-group formulations
* Real-world radar systems
