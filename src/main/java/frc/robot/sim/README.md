# Physics Sim Formulas

This document contains information and sources for the formulas used in the project.

## 1. Module Chassis Acceleration

### Variables

- **x**: x position
- **y**: y position
- **v_x**: linear velocity x component
- **v_y**: linear velocity y component
- **a_x**: linear acceleration x component
- **a_y**: linear acceleration y component
- **t**: timestep duration
- **θ**: drivetrain heading
- **ω**: drivetrain angular velocity
- **α**: drivetrain angular acceleration
- **m**: drivetrain mass

### Formulas

- **Position Update**:
  - x_{k+1} = x_k + v_{x_k} * t + (1/2) * a_{x_k} * t^2
  - y_{k+1} = y_k + v_{y_k} * t + (1/2) * a_{y_k} * t^2

- **Velocity Update**:
  - v_{x_{k+1}} = v_{x_k} + a_{x_k} * t
  - v_{y_{k+1}} = v_{y_k} + a_{y_k} * t

- **Heading Update**:
  - θ_{k+1} = θ_k + ω_k * t
  - ω_{k+1} = ω_k + α_k * t

- **Acceleration Calculation**:
  - a_{x_k} = ΣF_{x_k} / m
  - a_{y_k} = ΣF_{y_k} / m
  - α_k = Στ_k / J

- **Torque Calculation**:
  - τ_k = r × F

- **Force Calculation**:
  - F = m * a

- **Angular Acceleration Calculation**:
  - α = τ / J

- **Linear Acceleration Calculation**:
  - a = F / m

### Usage in code

In the simulation, these formulas are used to update the state of the robot's drivetrain at each timestep. For example, given the current state of the robot and the forces applied by the wheels, the new position, velocity, and heading of the robot can be calculated using the above formulas.
