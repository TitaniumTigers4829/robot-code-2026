# Physics Sim Formulas

This document describes the physics models used in the sim engine. The sim runs at a fixed sub-tick rate (default: 5 sub-ticks per 20ms robot loop), with dyn4j handling rigid body integration between ticks. This document covers the force/torque models applied on top of dyn4j, not the integrator internals.

---

## 1. Coordinate System

WPILib field convention throughout:
- X = 0 at Blue Alliance wall, +X toward Red Alliance wall
- Y = 0 at Audience-side wall, +Y toward Scoring Table wall
- Z = 0 at carpet, +Z up
- θ = heading, CCW positive (standard math convention)

---

## 2. Swerve Module Propulsion Forces

Each swerve module computes a propulsion force vector applied to the chassis at the module's offset position.

### Variables

| Symbol | Description |
|--------|-------------|
| `φ_i` | Steer angle of module `i` in world frame (`steerAngle + robotHeading`) |
| `τ_drive` | Drive motor torque at the wheel (`driveTorque / wheelRadius`) |
| `F_grip` | Maximum grip force (`normalForce × CoF`) |
| `F_prop` | Propelling force applied to chassis |
| `r_i` | Module position vector from chassis center, rotated to world frame |
| `F_i` | Propulsion force vector for module `i` |

### Grip limit (skid detection)

```
F_grip = m_robot/N_modules × g × CoF

if |τ_drive / r_wheel| > F_grip:
    F_prop = F_grip × sign(τ_drive)     ← skidding
else:
    F_prop = τ_drive / r_wheel           ← normal traction
```

Source: Coulomb friction model. `CoF` is configured per wheel compound in `SimSwerveModuleConfig.WheelCof`.

### Force vector decomposition

```
F_i = [ F_prop × cos(φ_i),  F_prop × sin(φ_i) ]
```

### Off-center force → torque conversion

For a force `F` applied at position `r` from chassis center:

```
F_center    = F                              (translational component)
τ_from_F    = r × F  =  r_x×F_y - r_y×F_x  (torque contribution)
```

This is implemented in `PhysicsMass.forcesDueToOffsetForces()`. The net chassis force and torque are the sum over all four modules.

---

## 3. Swerve Module Friction Forces

Friction opposes any wheel velocity component that is not aligned with the drive direction. This models the lateral ("scrub") friction that prevents the robot from sliding sideways.

### Unwanted velocity calculation

```
v_tangential_i = ω_chassis × |r_i|  (magnitude of tangential velocity at module i)
direction_tangential = angle(r_i) + robotHeading + 90°

v_module_world = [ v_chassis_x,  v_chassis_y ]
               + v_tangential × [ cos(dir_tangential),  sin(dir_tangential) ]

v_drive_i = driveVelocity × [ cos(φ_i),  sin(φ_i) ]   (where robot is trying to go)

v_unwanted_i = v_module_world - v_drive_i
```

### Friction force (sign-only model)

```
F_friction_i = F_grip × [ -sign(v_unwanted_x),  -sign(v_unwanted_y) ]
```

This is a simplified Coulomb model — full grip force opposes any nonzero unwanted velocity component. A more accurate model would use `tanh` or a continuous approximation near zero.

### Anti-reversal clamp

The total friction acceleration is clamped so it cannot reverse the chassis velocity — it can only decelerate, not accelerate in the opposite direction:

```
a_friction_clamped = clamp(a_friction, -v_chassis/dt)
```

This prevents the friction model from causing oscillation around zero velocity.

---

## 4. Rotor Inertia Interpolation

Effective rotor inertia felt by the chassis depends on whether the modules are primarily translating or rotating the chassis. The two limiting cases are:

```
J_translating = (m_chassis / N_modules) × r_wheel²
J_rotating    = (J_chassis / (N_modules × |r_module|²)) × r_wheel²
```

The actual rotor inertia is interpolated based on the propulsion force decomposition:

```
translationRatio = |F_xy_total| / |F_total|

J_rotor = J_translating × translationRatio
        + J_rotating    × (1 - translationRatio)
```

Where `F_xy_total` is the net translational propulsion force magnitude and `F_total` is the sum of all module propulsion force magnitudes.

Source: Derived from rigid body dynamics. When all forces produce pure translation, the effective inertia is dominated by linear mass. When forces produce pure rotation (spinning in place), the effective inertia is dominated by moment of inertia.

---

## 5. Gyro Drift Model

The gyro simulation models three noise sources added to the true angular velocity:

### Motionless drift

```
if |ω_actual| < 0.001 rad/s:
    drift_motionless = 0
else:
    drift_motionless = averageDriftingMotionless
                     = config.averageDriftingIn30SecsMotionlessDeg / 30s
```

Pigeon 2 spec (from official user guide):
- No-motion: **0.12 deg/hour = 0.001 deg/30sec**
- In-motion: **0.4 deg/min**

The stationary gate prevents drift accumulation while the robot is at rest. This was the source of the stationary odometry drift bug — the gate was previously missing.

### Velocity measurement noise

```
noise = ω_actual × Normal(0, σ_velo)
```

Where `σ_velo = config.velocityMeasurementStandardDeviationPercent`.

### Impact drift

Triggered when instantaneous angular acceleration exceeds a threshold (500 rad/s²):

```
if |α| > START_DRIFTING:
    drift_impact = DRIFT_COEFFICIENT × sign(α) × (α / START_DRIFTING) / dt
else:
    drift_impact = 0
```

Where `α = (ω_current - ω_last) / dt` and `DRIFT_COEFFICIENT = 1 rad` per multiple of threshold.

### Total reported angular velocity

```
ω_reported = ω_actual + drift_motionless + drift_impact + noise
```

### Linear acceleration (for IMU)

Computed from successive twist differentials:

```
a_x = (v_x_current - v_x_last) / dt
a_y = (v_y_current - v_y_last) / dt
```

---

## 6. Inter-Session Game Piece Decay (InFlight state)

Game pieces in `InFlight` state update each sub-tick using projectile dynamics:

```
v_{k+1} = ProjectileDynamics.calculate(dt, v_k)   // applies drag + gravity
pose_{k+1} = pose_k.exp(Twist3d(dt×v_x, dt×v_y, dt×v_z, 0, 0, 0))
```

Landing detection (Z < 0): piece transitions to `OnField` with horizontal velocity damped by `landingDampening` coefficient (0.3 for FUEL).

Scoring detection: piece checks if its 3D position intersects any `GamePieceTarget` bounding box. On intersection, transitions to `LIMBO`.

---

## 7. Motor Mechanism Model (SimMechanism)

Each drive/steer motor is modeled as a DC motor with:
- Back-EMF from rotor velocity
- Gear ratio
- Rotor inertia
- Static and kinetic friction
- Optional noise (Gaussian, added to output)
- Torque/current/voltage limits

The motor state update per sub-tick:

```
V_back_emf = k_v × ω_rotor
I = (V_applied - V_back_emf) / R_motor
τ_motor = k_t × I
τ_net = τ_motor - τ_friction - τ_load
α_rotor = τ_net / J_rotor
ω_rotor_{k+1} = ω_rotor_k + α_rotor × dt
θ_rotor_{k+1} = θ_rotor_k + ω_rotor_k × dt
```

Motor constants (`k_v`, `k_t`, `R`) are provided by WPILib's `DCMotor` model.

---

## 8. Chassis Rigid Body Integration

dyn4j handles position/velocity integration given the applied forces and torques. The sim applies forces via `chassis.applyForce()` and `chassis.applyTorque()` each sub-tick. dyn4j then solves contact constraints and integrates:

```
// Handled by dyn4j internally each world.updatev(dt) call:
v_{k+1} = v_k + (F_net / m) × dt
ω_{k+1} = ω_k + (τ_net / J) × dt
x_{k+1} = x_k + v_{k+1} × dt
θ_{k+1} = θ_k + ω_{k+1} × dt
```

Contact resolution (bumper CoF = 0.65, CoR = 0.005) is handled by dyn4j's constraint solver. These values model bumper-on-field-element contact as high-friction, low-bounce.

### Odometry twist extraction

```
dXY    = chassis.getChangeInPosition()     // from dyn4j body
dTheta = -chassis.getAngularVelocity() × dt

twist = Twist2d(dXY.x, dXY.y, dTheta)
```

The sign inversion on `dTheta` converts from dyn4j's CW-positive convention to WPILib's CCW-positive convention.

---

## 9. Timing

```
robotLoopPeriod = 0.020s  (50 Hz)
subTicksPerPeriod = 5
dt = robotLoopPeriod / subTicksPerPeriod = 0.004s  (250 Hz physics rate)
```

Higher physics rate reduces integration error, especially important for fast collision events and accurate friction modeling near zero velocity.

---

## 10. Sources

| Model | Source |
|-------|--------|
| Swerve propulsion / friction | First-principles rigid body dynamics |
| Rotor inertia interpolation | Derived from mass/MoI decomposition |
| Coulomb friction (grip limit) | Classical friction model |
| Gyro drift — Pigeon 2 specs | CTRE Pigeon 2 User Guide (official) |
| Motor model (DCMotor) | WPILib `DCMotor` class |
| Projectile dynamics | `ProjectileUtil.ProjectileDynamics` (internal) |
| Rigid body integration | dyn4j physics engine |
| Bumper CoF / CoR | Empirical estimates; CoF=0.65, CoR=0.005 |
