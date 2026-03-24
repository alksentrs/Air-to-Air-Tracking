# Implementation Plan & Checklist

## Airborne Radar Tracking System

---

## 1. Overview

This document defines the **step-by-step implementation plan** for the airborne radar tracking system.

Each stage includes:

* Objectives
* Tasks
* Validation criteria
* Completion checklist

This document is intended to be used interactively during development.

---

## 2. General Guidelines

* Follow MATLAB OOP (classdef)
* Keep classes modular and testable
* Avoid hardcoded parameters
* Validate each step before proceeding
* Use plots for debugging and validation

---

# ================================

# STAGE 1 — DYNAMIC SIMULATION

# ================================

## 3. Objective

Implement a ground truth simulator for:

* Aircraft circular motion
* Drone constant velocity motion

---

## 3.1 Tasks

### 3.1.1 Simulation Configuration

* [ ] Create `SimulationConfig` class
* [ ] Define:

  * [ ] simulation time
  * [ ] time step (dt)
  * [ ] number of drones
  * [ ] initial conditions

---

### 3.1.2 Aircraft Model

* [ ] Create `AircraftModel` class
* [ ] Implement state:

  * [ ] position ( p_a )
  * [ ] heading ( θ_a )
* [ ] Implement propagation:

  * [ ] angular rate ( ω = v/R )
  * [ ] circular motion update

Validation:

* [ ] Trajectory forms a circle
* [ ] Speed remains constant

---

### 3.1.3 Drone Model

* [ ] Create `DroneModel` class
* [ ] Implement:

  * [ ] position
  * [ ] velocity
* [ ] Constant velocity propagation

Validation:

* [ ] Trajectories are straight lines
* [ ] Velocity is constant

---

### 3.1.4 Simulation Engine

* [ ] Create `Simulator` class
* [ ] Implement time loop
* [ ] Store:

  * [ ] aircraft state history
  * [ ] drone states history

---

### 3.1.5 Visualization

* [ ] Create `TrajectoryVisualizer`
* [ ] Plot:

  * [ ] aircraft trajectory
  * [ ] drone trajectories
  * [ ] heading direction

---

## 3.2 Completion Criteria

* [ ] Aircraft trajectory is circular
* [ ] Drone trajectories are linear
* [ ] Visualization is correct
* [ ] No numerical instability

---

# ================================
# STAGE 2 — RADAR MEASUREMENTS
# ================================

## 4. Objective

Generate radar detections (range and azimuth) from the perspective of an aircraft-mounted radar.

The radar is **not forward-looking**. Instead:

- The radar line of sight is **perpendicular to the aircraft instantaneous velocity**
- The radar points toward the **inner side of the circular trajectory**
- The radar is therefore modeled as an **inward side-looking radar**

Additionally, detections are not guaranteed: targets are only detected if they lie within the radar **field of view (FOV)**.

---

## 4.1 Aircraft Motion and Revisit Time

Given:

- Aircraft speed: \( v_a = 100 \, \text{m/s} \)
- Radius: \( R = 1000 \, \text{m} \)

Angular rate:

\[
\omega_a = \frac{v_a}{R} = 0.1 \, \text{rad/s}
\]

Orbit period:

\[
T = \frac{2\pi R}{v_a} \approx 62.8 \, \text{s}
\]

Implication:

- Neglecting drone motion, the radar revisits the same geometry approximately every **62.8 seconds**
- Detection patterns should exhibit this periodic behavior

---

## 4.2 Radar Frame Definition

The radar frame is **not aligned with aircraft heading**.

Define:

- \( \hat{t} \): unit tangent vector (aircraft velocity direction)
- \( \hat{n}_{in} \): inward normal vector (toward circle center)

Radar boresight:

\[
\hat{b}_{radar} = \hat{n}_{in}
\]

Properties:

- \( \hat{b}_{radar} \perp \hat{t} \)
- Radar always points toward the center of the circular trajectory
- Radar frame rotates continuously with aircraft motion

---

## 4.3 Processing Pipeline (Per Time Step)

For each target and each time step:

1. Compute relative position in inertial frame:
   \[
   p_{rel} = p_t - p_a
   \]

2. Transform to radar frame:
   \[
   p_{rel}^{radar}
   \]

3. Check detection (FOV + range)

4. If detected:
   - Compute measurements \( \rho, \phi \)
   - Add noise

5. If not detected:
   - No measurement is generated

---

## 4.4 Tasks

### 4.4.1 Coordinate Transformation

- [ ] Create `CoordinateUtils`
- [ ] Implement:
  - [ ] SO(2) rotation matrices
  - [ ] inertial → aircraft frame
  - [ ] aircraft → radar frame
  - [ ] inward normal vector computation

Validation:
- [ ] Tangent and inward normal are orthogonal
- [ ] Radar boresight always points inward
- [ ] Transformations are consistent

---

### 4.4.2 Radar Frame Construction

- [ ] Define radar frame based on aircraft motion
- [ ] Align radar x-axis with inward normal
- [ ] Ensure consistent rotation over time

Validation:
- [ ] Radar orientation follows aircraft motion
- [ ] Boresight points toward center at all times

---

### 4.4.3 Relative Position

- [ ] Compute:
  \[
  p_{rel} = p_t - p_a
  \]

- [ ] Transform:
  \[
  p_{rel}^{radar}
  \]

---

### 4.4.4 Detection Logic (Field of View)

Targets are only detected if inside the radar field of view.

Define:

- \( \Theta_{FOV} \): total angular aperture
- \( \alpha = \Theta_{FOV}/2 \): half-angle
- \( \rho_{min}, \rho_{max} \): range limits

From radar-frame coordinates:

\[
\rho = \sqrt{x_r^2 + y_r^2}, \quad
\phi = \mathrm{atan2}(y_r, x_r)
\]

Detection condition:

\[
|\phi| \le \alpha
\]

and

\[
\rho_{min} \le \rho \le \rho_{max}
\]

Tasks:

- [ ] Add FOV parameters to `RadarSensor`
- [ ] Implement angular gating
- [ ] Implement range gating
- [ ] Return detection flag

Validation:

- [ ] Targets outside sector are not detected
- [ ] Targets inside sector are detected
- [ ] Detection pattern matches radar geometry

---

### 4.4.5 Measurement Model

For detected targets:

- [ ] Compute:
  \[
  \rho = \|p_{rel}^{radar}\|
  \]
  \[
  \phi = \mathrm{atan2}(y_r, x_r)
  \]

Validation:

- [ ] Range is always positive
- [ ] Azimuth within [-π, π]
- [ ] Geometry consistent with radar orientation

---

### 4.4.6 Noise Model

- [ ] Add Gaussian noise:
  - [ ] \( \sigma_{range} \)
  - [ ] \( \sigma_{azimuth} \)
- [ ] Make configurable

\[
\rho_{meas} = \rho + n_\rho
\]
\[
\phi_{meas} = \phi + n_\phi
\]

---

### 4.4.7 Measurement Logging

- [ ] Store per target:
  - [ ] detection flag
  - [ ] true measurements
  - [ ] noisy measurements
- [ ] Represent missed detections explicitly

---

### 4.4.8 Visualization

- [ ] Plot:
  - [ ] range vs time
  - [ ] azimuth vs time
  - [ ] detection timeline (visible / not visible)

- [ ] Optional:
  - [ ] radar boresight direction
  - [ ] FOV sector visualization
  - [ ] revisit cycles (~62.8 s)

---

## 4.5 Completion Criteria

- [ ] Radar boresight is perpendicular to velocity
- [ ] Radar points inward to circular trajectory
- [ ] Detection respects FOV constraints
- [ ] Measurements match geometry
- [ ] Noise behaves correctly
- [ ] Angle wrapping handled correctly
- [ ] Periodic revisit (~62.8 s) observable when drone motion is negligible

# ================================

# STAGE 3 — TRACKING (EKF)

# ================================

## 5. Objective

Estimate drone states from radar measurements using EKF.

---

## 5.1 Tasks

### 5.1.1 State Definition

* [ ] Define state:

  * [ ] ( x = [px, py, vx, vy]^T )

---

### 5.1.2 Dynamic Model

* [ ] Implement matrix F
* [ ] Implement prediction:

  * [ ] state propagation
  * [ ] covariance propagation

---

### 5.1.3 Measurement Function

* [ ] Implement nonlinear function:

  * [ ] ( h(x) )
* [ ] Include:

  * [ ] aircraft position
  * [ ] frame transformation

---

### 5.1.4 Jacobian

* [ ] Derive ( H = ∂h/∂x )
* [ ] Implement analytically

Validation:

* [ ] Compare with numerical Jacobian

---

### 5.1.5 EKF Implementation

* [ ] Create `EKFTracker`
* [ ] Implement:

  * [ ] predict()
  * [ ] update()
  * [ ] innovation
  * [ ] Kalman gain

---

### 5.1.6 Angle Handling

* [ ] Normalize azimuth innovation
* [ ] Avoid discontinuities

---

### 5.1.7 Logging

* [ ] Store:

  * [ ] estimated states
  * [ ] covariance
  * [ ] innovations

---

### 5.1.8 Visualization

* [ ] Plot:

  * [ ] true vs estimated trajectory
  * [ ] position error
  * [ ] residuals

---

## 5.2 Completion Criteria

* [ ] Filter converges
* [ ] Errors decrease over time
* [ ] No divergence
* [ ] Innovation is bounded

---

# ================================

# 6. FINAL VALIDATION

# ================================

* [ ] End-to-end simulation works
* [ ] Multiple drones supported
* [ ] Results reproducible
* [ ] Parameters configurable

---

# ================================

# 7. FUTURE EXTENSIONS (OPTIONAL)

# ================================

* [ ] UKF implementation
* [ ] Particle Filter
* [ ] Multi-target tracking
* [ ] Data association
* [ ] Clutter modeling
* [ ] Lie-group filters (SE(2), SE(3))

---

# ================================

# 8. DEVELOPMENT STATUS

# ================================

Stage 1: ☐ Not started ☐ In progress ☐ Completed
Stage 2: ☐ Not started ☐ In progress ☐ Completed
Stage 3: ☐ Not started ☐ In progress ☐ Completed

---

## Notes

Use this document as a **live checklist** during implementation.

Each item should only be marked as complete after validation.
