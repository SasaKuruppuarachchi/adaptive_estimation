# Project Proposal: Real-time Adaptive Estimation using Temporal Predictive Coding (tPC)

## Team Members:

Sasanka Kuruppu Arachchige
Nicklas Fienda

## Mentors:

Gökhan Alcan
Azwirman Gusraldi

# 1. Introduction

State estimation is an essential process for controlling dynamic systems. Traditional state estimators, such as Kalman filters, rely on prior knowledge of system dynamics to perform accurate estimation. However, in systems with complex dynamics or rapidly changing conditions, such models can be difficult to develop or may fail to adapt to system changes.

This project proposes using Temporal Predictive Coding (tPC) as an adaptive state estimation approach. tPC is a learning-based method that identifies system dynamics and performs state estimation. We hypothesize that this approach will match or surpass the performance of traditional state estimators by adapting to changes in both system dynamics and environmental factors in real-time.

# 2. Problem Statement

**Traditional state estimators face several challenges:**

- They require prior knowledge of system dynamics, which can be difficult to obtain in complex systems.
- Even when dynamic models are available, they do not adapt effectively to changes in the system.

This project seeks to address these limitations by utilising a novel, learning-based method that can dynamically adjust to system changes.

# 3. Proposal: Temporal Predictive Coding (tPC)

**Temporal Predictive Coding offers a potential solution to the aforementioned challenges:**

- Learning-based method: tPC learns the dynamics of the system and continuously updates the state estimates along with system dynamics, unlike static models.
- Adaptive: tPC can adapt to changes in both system and object dynamics, making it suitable for real-time applications.
- Performance: The proposed method aims to match or exceed the performance of traditional estimators like the Kalman filter.

# 4. Conceptual Design

**The design includes several key components:**

**Control (LQR or PI):** Control algorithms like Linear Quadratic Regulator (LQR) or Proportional-Integral (PI) will be used to adjust system behavior based on state estimates.

**State Estimation:** The system state $X=x,θ,\dot{x},\dot{\theta}$ will be estimated using the tPC framework.

**Actuation (Motor Control):** The control signal will be used to drive actuators, with Pulse Width Modulation (PWM) applied for motor control.

**Sensor Measurements:** Sensor data, $θ_1,θ_2,\dot{\theta_1}, \dot{\theta_2}$, will provide real-time feedback to the state estimator for continuous adjustment.

The conceptual design will be implemented and evaluated using the Furuta Pendulum model, which offers a test bed for validating the adaptive nature of tPC.

# 5. Project Goals

The primary goal of this project is to implement a tPC-based adaptive state estimator, and to validate its performance in both simulation and real-world hardware environments.

# 6. Objectives
## Phase 1: Simulation

- Simulator Setup: Use Nvidia Isaac Sim to simulate the Furuta Pendulum system.
- State Estimation: Replicate the results of traditional state estimators (e.g., Kalman Filter) and compare these to tPC-based estimation.
- Test LQR and PID controllers in simulation.
- Evaluate system adaptability by simulating changes in:
  - Physical parameters (e.g., gravity).
  - Dynamical parameters (e.g., pendulum mass).
  - Sensor modalities and configurations.

## Phase 2: Real-World Deployment

- Physical Device: Construct and test the system on real-world hardware using the Furuta Pendulum setup.
- Comparison: Evaluate the adaptability of Kalman Filter and tPC to real-world changes (e.g., sensor noise, parameter drift).

# 7. Conclusion

This project aims to push the boundaries of real-time state estimation by introducing a learning-based approach that adapts to changing dynamics. By integrating Temporal Predictive Coding into control systems, we aim to demonstrate an improvement over traditional methods in dynamic environments.
