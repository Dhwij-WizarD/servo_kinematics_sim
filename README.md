# 4-DOF Robotic Arm Inverse Kinematics & Teleoperation (MuJoCo)

An interactive 4-DOF articulated robotic arm simulation built in **Python** and **MuJoCo**, utilizing closed-form geometric inverse kinematics (IK) for real-time 3D Cartesian positioning and end-effector pitch control via non-blocking terminal keyboard input.

---

## Features

* **Closed-Form Geometric IK Solver:** Uses pure analytical trigonometry (Law of Cosines) to solve for all four joint angles ($\theta_1, \theta_2, \theta_3, \theta_4$) in real time without iterative matrix solvers.
* **4-DOF Position & Orientation Control:** Independently controls end-effector $(X, Y, Z)$ coordinates while maintaining a desired tool pitch angle.
* **Workspace & Singularity Guards:** Enforces inner Z-axis cylindrical bounds, minimum/maximum radial reach clamping, and joint angle limits to prevent gimbal flips and full-extension singularities.
* **Live Coordinate Frame Visualization:** Renders local RGB coordinate axes ($X=\text{Red}, Y=\text{Green}, Z=\text{Blue}$) and rotation shafts at every joint in the MuJoCo 3D viewer.
* **Non-Blocking Keyboard Teleoperation:** Uses Linux `termios`/`tty` cbreak mode for instant single-keypress control from the terminal (no `Enter` key required) with a live single-line joint angle readout.

---

## Kinematic Model

The arm consists of three active links mounted on a ground-level yaw base ($z = 0$):

| Joint | Name | Axis | Description |
| :--- | :--- | :--- | :--- |
| **$\theta_1$** | Base Yaw | $Z$ | Rotates the entire arm toward target $(X, Y)$ |
| **$\theta_2$** | Shoulder Pitch | $Y$ | Interior angle between the ground plane and Link 1 ($L_1 = 0.20\text{ m}$, Cyan) |
| **$\theta_3$** | Elbow Pitch | $Y$ | Interior angle between Link 1 ($L_1$) and Link 2 ($L_2 = 0.20\text{ m}$, Pink) |
| **$\theta_4$** | Wrist Pitch | $Y$ | Interior angle between Link 2 ($L_2$) and Link 3 ($L_3 = 0.10\text{ m}$, Red) |

---

## Project Structure

```text
mujoco_4dof_ik/
├── model/
│   └── arm_4dof.xml        # MJCF robot definition, joint frames, and position servos
├── controller.py           # Analytical 4-DOF IK solver and terminal keyboard controller
├── run.sh                  # Environment activation and launch script
└── README.md