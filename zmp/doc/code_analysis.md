# GankenKun Walking Pattern Generator: Code Analysis & Algorithm Review

This document provides a comprehensive code analysis and algorithm review of the GankenKun walking pattern generation system, implemented in Python for the PyBullet physics simulator.

## 1. Architecture Overview

The system is designed to enable a bipedal robot (GankenKun) to walk towards a given 2D goal `(x, y, theta)`. It employs a classic hierarchical bipedal locomotion framework:
1. **Foot Step Planning:** Determines where the robot should place its feet to reach the destination.
2. **Preview Control (ZMP tracking):** Generates a smooth Center of Mass (CoM) trajectory that ensures dynamic balance during walking.
3. **Trajectory Interpolation:** Computes the temporal 6D pose of both feet as they lift and swing to the next step.
4. **Inverse Kinematics (IK):** Translates the 3D Cartesian coordinates of the CoM and feet into individual joint angles.

The primary modules handling this logic reside in the `GankenKun/` directory:
- `foot_step_planner.py` (and `v2`)
- `preview_control.py` (and `v2`)
- `kinematics.py`
- `walking.py` (and `v2`)

---

## 2. Algorithm Review

### 2.1 Foot Step Planner (`foot_step_planner.py`)
**Goal:** Generate a sequence of discrete footstep placements `[time, x, y, theta, leg]` from the current state to the goal state.
**Algorithm Details:**
- **Linear Interpolation with Constraints:** The planner evaluates the distance to the goal `(x, y, theta)` and divides it into steps based on predefined limits (`max_stride_x`, `max_stride_y`, `max_stride_th`).
- **Alternating Legs:** It tracks the current support leg and switches between `'left'` and `'right'` for each step, naturally applying a lateral `width` offset so the feet do not collide.
- **Phases:** The sequence initializes with a `'start'` phase (both feet on ground), transitions into a looping walking phase, and culminates in a `'stop'` phase where both feet align at the goal location. 

### 2.2 Preview Control for CoM Trajectory (`preview_control.py`)
**Goal:** Derive a Center of Mass (CoM) trajectory that maintains dynamic stability.
**Algorithm Details:**
- **Linear Inverted Pendulum Model (LIPM):** The robot is modeled as an inverted pendulum with a constant height.
- **Zero Moment Point (ZMP):** The desired ZMP is defined by the center of the support foot (or between the feet during double support). 
- **Discrete-time Algebraic Riccati Equation (DARE):** The preview controller uses Future ZMP references (previewing upcoming footsteps) to compute an optimal CoM trajectory. It defines a cost function weighing tracking error (difference between actual and reference ZMP) and control effort.
- **Implementation Note:** The script constructs a state-space system (`A_d`, `B_d`, `C_d`) and originally used `control.dare` to solve for the optimal feedback gain matrices. Due to numerical instabilities (e.g., `SlycotArithmeticError`), it is strictly recommended to use robust solvers like `scipy.linalg.solve_discrete_are`.

### 2.3 Inverse Kinematics (`kinematics.py`)
**Goal:** Convert 6D Cartesian target poses of the left and right feet (relative to the waist) into joint angles.
**Algorithm Details:**
- **Analytical IK:** Instead of computationally expensive Jacobian-based numerical IK, the solver uses an analytical (geometric) approach, taking advantage of the robot's leg structure (standard 6-DOF leg with yaw-roll-pitch at the hip, pitch at the knee, and pitch-roll at the ankle).
- **Calculations:** 
  - Inverse trigonometric functions (`math.atan2`, `math.acos`) calculate joint angles based on leg link lengths (`L1`, `L2`, `L3`).
  - Mimic joints are explicitly handled (e.g., `left_shin_pitch_mimic_link` mirroring `knee_pitch`) to accurately reflect the mechanical linkages in GankenKun's design.

### 2.4 Walking Pattern Generator (`walking.py`)
**Goal:** Orchestrate the planners and controllers to output frame-by-frame joint angles.
**Algorithm Details:**
- **State Management:** Manages the active footstep plan and pulls the upcoming CoM state from `preview_control`.
- **Foot Swing Trajectory:** Generates the swinging foot trajectory using simple linear interpolation combined with a parabolic/triangular lifting profile to a predefined `foot_height`.
- **Integration:** Calls the IK solver per timestep with the updated CoM (which dictates the waist frame) and the interpolated foot poses, returning the `joint_angles` array that PyBullet's `setJointMotorControl2` applies to the simulation.

---

## 3. Code Analysis & Interdependencies

- **`GankenKun.py` (Main Loop):**
  This acts as the physics simulation environment (PyBullet). It loads the URDF models, initializes the environment, sets up the objects (`preview_control`, `foot_step_planner`, `walking`), and runs the `p.stepSimulation()` loop.
- **Modularity:** The codebase separates physics/simulation from the mathematical controllers. `GankenKun.py` handles PyBullet, while the `GankenKun/` directory handles pure math.
- **Limitations:** The footstep planner uses straightforward geometric interpolation without considering obstacles. The swing foot trajectory is linear and not a smooth spline (like Bezier), which can cause jerky velocity profiles at the apex of the step.

## 4. Conclusion
The GankenKun framework provides a robust educational template for ZMP-based bipedal locomotion. By chaining a geometric step planner with DARE-based preview control and analytical IK, it allows a simulated humanoid robot to walk stably toward dynamic targets without falling.
