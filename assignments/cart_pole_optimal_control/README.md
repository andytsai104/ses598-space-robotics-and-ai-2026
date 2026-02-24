
# Cart-Pole LQR Optimal Control

**Author:** Andy Tsai  
**Course:** SES 598 – Space Robotics and AI (Spring 2026)

---

## Overview

This project analyzes and tunes a **Linear Quadratic Regulator (LQR)** controller for an inverted cart-pole system subject to earthquake disturbances.

The objectives are:

- Stabilize the pole in the upright position  
- Keep the cart within ±2.5 m physical constraints  
- Maintain stable behavior under external disturbances  

A systematic **one-parameter-at-a-time scaling strategy** is used to isolate the effects of each LQR weight in the Q and R matrices.

---

## System Model

State vector:

x = [x, x_dot, theta, theta_dot]^T

Control input:

u = cart force

The LQR controller minimizes:

J = ∫ (x^T Q x + u^T R u) dt

Where:

- **Q** penalizes state deviations  
- **R** penalizes control effort  

---

## Package Structure

```text
cartpole_control/
├── cart_pole_optimal_control/
├── config/
├── launch/
├── media/
│   ├── default_params/
│   ├── 100x_q1/
│   ├── 100x_q3/
│   ├── 0.01x_r1/
│   ├── optimal_params/
│   └── comparison_plots/
├── models/
├── resources/
├── rviz/
├── test/
├── README.md
├── assignment_details.md
├── package.xml
├── setup.cfg
└── setup.py
```

Each folder under `media/` contains:

- `sim.gif` – Simulation 
- `plots.png` – Results Plots 
- `terminal_log.png` – Terminal output


---

## Baseline Parameters (Default)

```text
Q = diag([1.0, 1.0, 10.0, 10.0])    # State Cost
R = [[1.0]]                         # Control Cost
```

### Baseline Observations

- Stable under moderate disturbances  
- Noticeable cart drift under strong forcing  
- Moderate control effort  
- Acceptable angle recovery  

---

## Tuning Strategy

From physical intuition:

- Cart position (q1) affects boundary constraint enforcement  
- Pole angle (q3) is most critical for stability  
- Control cost (r1) affects aggressiveness  

Experiments performed:

1. 100× q1  
2. 100× q3  
3. 0.01× r1
4. 10xOptimal parameters

Each parameter was scaled independently to isolate behavior changes.

---

## Running the Simulation

```bash
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

---

## Experiment 1 — 100× q1

```text
Q = diag([100.0, 1.0, 10.0, 10.0])  
R = [[1.0]]
```

---

## Experiment 2 — 100× q3

```text
Q = diag([1.0, 1.0, 1000.0, 10.0])  
R = [[1.0]]
```

---

## Experiment 3 — 0.01× r1

```text
Q = diag([1.0, 1.0, 10.0, 10.0])  
R = [[0.01]]
```

---

## Final Tuned Parameters

```text
Q = diag([20.0, 1.0, 200.0, 20.0])  
R = [[0.1]]
```

---


## Notes

* All plots are generated automatically and saved as PNG files
* Experiments are fully reproducible using the provided launch files

---

## License

For academic use only (course assignment).
