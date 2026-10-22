#!/usr/bin/env python3
"""Offline visual test for the RMRC trajectory in real_test.py."""

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry
import swift
from spatialmath import SE3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from real_test import circle_traj

RADIUS = 0.06
START_HEIGHT_OFFSET = 0.2
STEPS = 200
DT = 0.05


def build_test_trajectory():
    robot = rtb.models.UR3()
    end_effector = robot.ee_links[0]
    q_home = robot.qz.copy()
    start_pose = robot.fkine(q_home, end=end_effector)
    raised_pose = SE3.Trans(0, 0, START_HEIGHT_OFFSET) * start_pose
    ik_result = robot.ikine_LM(
        raised_pose,
        q0=q_home,
        end=end_effector,
        joint_limits=True,
    )
    if not ik_result.success:
        raise RuntimeError(f"Could not find raised UR3 start pose: {ik_result.reason}")

    q_start = ik_result.q
    trajectory = np.asarray(
        circle_traj(
            robot,
            q_start,
            radius=RADIUS,
            steps=STEPS,
            dt=DT,
            end=end_effector,
        )
    )

    expected_shape = (STEPS + 1, robot.n)
    if trajectory.shape != expected_shape:
        raise RuntimeError(
            f"Expected trajectory shape {expected_shape}, got {trajectory.shape}"
        )
    if not np.isfinite(trajectory).all():
        raise RuntimeError("Trajectory contains non-finite joint positions")
    if robot.qlim is not None:
        if np.any(trajectory < robot.qlim[0] - 1e-8) or np.any(
            trajectory > robot.qlim[1] + 1e-8
        ):
            raise RuntimeError("Trajectory exceeds the UR3 joint limits")

    positions = np.asarray(
        [robot.fkine(q, end=end_effector).t for q in trajectory]
    )
    return robot, trajectory, positions


def target_circle(start_position):
    theta = np.linspace(0, 2 * np.pi, STEPS)
    return start_position + np.column_stack(
        (
            RADIUS * (np.cos(theta) - 1),
            np.zeros_like(theta),
            RADIUS * np.sin(theta),
        )
    )


def print_summary(actual_positions, target_positions):
    errors = np.linalg.norm(actual_positions[1:] - target_positions, axis=1)
    print(f"Generated {STEPS + 1} joint samples for the UR3.")
    print(f"Maximum end-effector tracking error: {errors.max() * 1000:.3f} mm")
    print("This is an offline simulation; no ROS connection or physical motion is used.")


def show_plot(actual_positions, target_positions):
    figure = plt.figure()
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(
        target_positions[:, 0],
        target_positions[:, 1],
        target_positions[:, 2],
        "--",
        label="Requested circle",
    )
    axes.plot(
        actual_positions[:, 0],
        actual_positions[:, 1],
        actual_positions[:, 2],
        label="UR3 end-effector path",
    )
    axes.scatter(*actual_positions[0], marker="o", label="Start")
    axes.scatter(*actual_positions[-1], marker="x", label="End")
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_zlabel("Z (m)")
    axes.set_title("RMRC circular trajectory")
    axes.legend()
    axes.grid(True)

    bounds = np.vstack((actual_positions, target_positions))
    center = (bounds.max(axis=0) + bounds.min(axis=0)) / 2
    half_range = max(np.ptp(bounds, axis=0)) / 2 * 1.1
    axes.set_xlim(center[0] - half_range, center[0] + half_range)
    axes.set_ylim(center[1] - half_range, center[1] + half_range)
    axes.set_zlim(center[2] - half_range, center[2] + half_range)
    axes.set_box_aspect((1, 1, 1))
    plt.show()


def play_in_swift(robot, trajectory, actual_positions):
    environment = swift.Swift()
    environment.launch(realtime=True)
    try:
        robot.q = trajectory[0]
        environment.add(robot)

        circle_center = actual_positions[0] + np.array([-RADIUS, 0.0, 0.0])
        environment.set_camera_pose(
            circle_center + np.array([0.35, 0.5, 0.25]),
            circle_center,
        )

        # Show the requested path as a faint guide, then leave a brighter
        # trail behind the moving end effector during playback.
        for position in target_circle(actual_positions[0])[::5]:
            environment.add(
                geometry.Sphere(
                    radius=0.0015,
                    pose=SE3.Trans(*position),
                    color=[0.9, 0.7, 0.1, 0.6],
                    collision=False,
                )
            )

        environment.add(
            geometry.Sphere(
                radius=0.003,
                pose=SE3.Trans(*actual_positions[0]),
                color=[0.1, 0.4, 1.0, 1.0],
                collision=False,
            )
        )

        print("Playing the simulated trajectory in Swift.")
        for index, q in enumerate(trajectory[1:], start=1):
            robot.q = q
            environment.step(DT)

            if index % 4 == 0 or index == len(trajectory) - 1:
                environment.add(
                    geometry.Sphere(
                        radius=0.003,
                        pose=SE3.Trans(*actual_positions[index]),
                        color=[0.1, 0.4, 1.0, 1.0],
                        collision=False,
                    )
                )

        print("Playback complete. Close the Swift window to exit.")
        environment.hold()
    finally:
        environment.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("plot", "swift"),
        default="plot",
        help="show a 3D plot (default) or play the robot in Swift",
    )
    args = parser.parse_args()

    robot, trajectory, actual_positions = build_test_trajectory()
    target_positions = target_circle(actual_positions[0])
    print_summary(actual_positions, target_positions)

    if args.mode == "swift":
        play_in_swift(robot, trajectory, actual_positions)
    else:
        show_plot(actual_positions, target_positions)


if __name__ == "__main__":
    main()
