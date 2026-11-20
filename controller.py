import math
import select
import sys
import termios
import time
import tty
import mujoco
import mujoco.viewer

# Link lengths matching model/arm_4dof.xml
L1 = 0.20  # Cyan/Blue link length
L2 = 0.20  # Pink link length
L3 = 0.10  # Red link length

# Initial target [x, y, z, red_link_pitch_from_horizontal]
# pitch = -60 deg tilts the red link downward like in your sketch
target = [0.22, 0.00, 0.22, math.radians(-60.0)]


def solve_sketch_ik(x: float, y: float, z: float, tool_pitch: float):
    """
    Solves IK for your exact 4-DOF geometry:
      theta1 : Base yaw around Z
      theta2 : Interior angle between black ground and cyan/blue link (90 deg = vertical)
      theta3 : Interior angle between cyan/blue link and pink link
      theta4 : Interior angle between pink link and red link
    """
    # 1. Base yaw (theta1) + Z-axis singularity guard
    r_tip = math.hypot(x, y)
    if r_tip < 0.04:
        scale = 0.04 / max(r_tip, 1e-6)
        x *= scale
        y *= scale
        r_tip = 0.04
    theta1 = math.atan2(y, x)

    # 2. Backtrack along the Red link (L3) from the tip to find the Wrist joint (r_w, z_w)
    r_w = r_tip - L3 * math.cos(tool_pitch)
    z_w = z - L3 * math.sin(tool_pitch)
    r_w = max(0.02, r_w)
    z_w = max(0.02, z_w)

    # Clamp wrist reach within safe non-singular bounds [min_reach, max_reach]
    min_reach = 0.06
    max_reach = (L1 + L2) * 0.95  # 0.38m
    dist = math.hypot(r_w, z_w)
    if dist > max_reach:
        scale = max_reach / dist
        r_w *= scale
        z_w *= scale
        dist = max_reach
    elif dist < min_reach:
        scale = min_reach / max(dist, 1e-6)
        r_w *= scale
        z_w *= scale
        dist = min_reach

    # 3. Law of Cosines for interior angle theta3 (between Blue L1 and Pink L2)
    cos_theta3 = (L1**2 + L2**2 - dist**2) / (2.0 * L1 * L2)
    cos_theta3 = max(-1.0, min(1.0, cos_theta3))
    theta3 = math.acos(cos_theta3)

    # 4. Interior angle theta2 (between Black ground and Blue L1)
    gamma = math.atan2(z_w, r_w)
    cos_alpha = (L1**2 + dist**2 - L2**2) / (2.0 * L1 * dist)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)
    theta2 = gamma + alpha  # Elbow-up pose (like your sketch)

    # 5. Interior angle theta4 (between Pink L2 and Red L3)
    pink_elevation = theta2 + theta3 - math.pi
    theta4 = math.pi + (tool_pitch - pink_elevation)

    # Map geometric interior angles to MuJoCo joint coordinates (where 0 = straight up)
    q1 = theta1
    q2 = (math.pi / 2.0) - theta2
    q3 = math.pi - theta3
    q4 = math.pi - theta4

    return (q1, q2, q3, q4), (theta1, theta2, theta3, theta4)


def get_key_nonblocking() -> str:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).upper()
    return ""


def handle_terminal_input() -> bool:
    key = get_key_nonblocking()
    if not key:
        return True

    step = 0.015
    pitch_step = math.radians(5.0)
    updated = True

    if key == "W":
        target[0] += step  # +X
    elif key == "S":
        target[0] -= step  # -X
    elif key == "A":
        target[1] += step  # +Y
    elif key == "D":
        target[1] -= step  # -Y
    elif key == "E":
        target[2] += step  # +Z
    elif key == "Q":
        target[2] -= step  # -Z
    elif key == "T":
        target[3] += pitch_step  # Tilt red link up
    elif key == "G":
        target[3] -= pitch_step  # Tilt red link down
    elif key == "R":
        target[:] = [0.22, 0.00, 0.22, math.radians(-60.0)]
    elif key == "X":
        return False
    else:
        updated = False

    if updated:
        target[2] = max(0.02, target[2])
        target[3] = max(math.radians(-85), min(math.radians(85), target[3]))
        _, (t1, t2, t3, t4) = solve_sketch_ik(*target)
        sys.stdout.write(
            f"\rTip: [{target[0]:5.2f}, {target[1]:5.2f}, {target[2]:5.2f}]m | "
            f"theta1: {math.degrees(t1):5.1f}°  theta2: {math.degrees(t2):5.1f}°  "
            f"theta3: {math.degrees(t3):5.1f}°  theta4: {math.degrees(t4):5.1f}°   "
        )
        sys.stdout.flush()

    return True


def main():
    model = mujoco.MjModel.from_xml_path("model/arm_4dof.xml")
    data = mujoco.MjData(model)

    # Spawn directly in your sketch's starting pose (no initial jump)
    q_init, _ = solve_sketch_ik(*target)
    for i in range(4):
        data.qpos[i] = q_init[i]
        data.ctrl[i] = q_init[i]
    data.mocap_pos[0] = target[:3]
    mujoco.mj_forward(model, data)

    print("Controls (Terminal focused, no Enter needed):")
    print("  [W / S] : Target X (+ / -)   |  [A / D] : Target Y (+ / -)")
    print("  [E / Q] : Target Z (+ / -)   |  [T / G] : Red Link Angle (Up / Down)")
    print("  [R]     : Reset Pose         |  [X]     : Quit\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 1.0
            viewer.cam.elevation = -20
            viewer.cam.azimuth = 120
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

            while viewer.is_running():
                step_start = time.time()

                if not handle_terminal_input():
                    break

                data.mocap_pos[0] = target[:3]
                q_cmd, _ = solve_sketch_ik(*target)
                for i in range(4):
                    data.ctrl[i] = q_cmd[i]

                mujoco.mj_step(model, data)
                viewer.sync()

                time_until_next_step = model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nSimulation stopped.")


if __name__ == "__main__":
    main()