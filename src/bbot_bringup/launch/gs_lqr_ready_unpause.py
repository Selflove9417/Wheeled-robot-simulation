"""Single-step a paused GS-LQR world until its controllers and sensors are ready."""

import argparse
import subprocess
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import Imu, JointState

REQUIRED_CONTROLLERS = {
    "joint_state_broadcaster",
    "leg_position_controller",
    "wheel_effort_controller",
}


def world_control(world, request):
    result = subprocess.run(
        ["ign", "service", "-s", f"/world/{world}/control",
         "--reqtype", "ignition.msgs.WorldControl",
         "--reptype", "ignition.msgs.Boolean", "--timeout", "2000",
         "--req", request],
        capture_output=True, text=True, timeout=4,
    )
    return result.returncode == 0 and "data: true" in result.stdout


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("gs_lqr_startup_gate")
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    seen = {"imu": False, "joints": False}
    node.create_subscription(Imu, "/imu", lambda _: seen.__setitem__("imu", True), 10)

    def on_joints(msg):
        if {"link_004_joint", "link_007_joint"}.issubset(msg.name):
            seen["joints"] = True

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    deadline = time.monotonic() + args.timeout
    active = set()
    steps = 0
    pending = None
    try:
        while time.monotonic() < deadline:
            # Controller activation is applied by Gazebo's physics update.
            # Keep stepping even while a list_controllers request is pending.
            if not world_control(args.world, "pause: true multi_step: 1"):
                time.sleep(0.1)  # World service may still be starting.
                continue
            steps += 1
            rclpy.spin_once(node, timeout_sec=0.02)
            if pending is None:
                if client.service_is_ready():
                    pending = client.call_async(ListControllers.Request())
            elif pending.done():
                if pending.exception() is None:
                    active = {c.name for c in pending.result().controller
                              if c.state == "active"}
                pending = None

            if (REQUIRED_CONTROLLERS <= active and
                    seen["imu"] and seen["joints"]):
                # Let the balance node consume both inputs and issue a wheel command.
                for _ in range(5):
                    if not world_control(args.world, "pause: true multi_step: 1"):
                        raise RuntimeError("Gazebo single-step request failed")
                    rclpy.spin_once(node, timeout_sec=0.02)
                if not world_control(args.world, "pause: false"):
                    raise RuntimeError("Gazebo unpause request failed")
                print(f"GS-LQR ready after {steps + 5} physics steps; "
                      "controllers and sensors active; world running", flush=True)
                return 0
        missing = sorted(REQUIRED_CONTROLLERS - active)
        raise RuntimeError(f"startup timed out after {steps} steps; "
                           f"inactive controllers: {missing}; sensors: {seen}")
    except (RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"GS-LQR startup failed; physics remains paused: {exc}",
              file=sys.stderr, flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
