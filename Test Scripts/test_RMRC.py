#!/usr/bin/env python3
"""Offline Swift test of RMRC for the AR4 state-1 wheel pick-and-place."""

import argparse
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb
import swift
from spatialgeometry import Mesh
from spatialmath import SE3
from spatialmath.base import tr2angvec

from Robots.AR4_MK5.AR4_MK5 import AR4_MK5


PROJECT_ROOT = Path(__file__).resolve().parent
DT = 0.02
POSITION_TOLERANCE = 1.5e-3
ROTATION_TOLERANCE = 3e-3
DAMPING = 0.01
MAX_JOINT_STEP = 0.12


def pose_error(robot, q, target_world):
    actual_world = robot.fkine(q)
    position_error = target_world.t - actual_world.t
    angle, axis = tr2angvec(target_world.R @ actual_world.R.T)
    rotation_error = angle * axis
    return np.r_[position_error, rotation_error], actual_world


def solve_pose_rmrc(robot, q_start, target_world, label):
    """Track a Cartesian pose with damped-least-squares differential IK."""
    q = np.asarray(q_start, dtype=float).copy()

    for _ in range(120):
        error, _ = pose_error(robot, q, target_world)
        position_norm = np.linalg.norm(error[:3])
        rotation_norm = np.linalg.norm(error[3:])
        if (position_norm <= POSITION_TOLERANCE
                and rotation_norm <= ROTATION_TOLERANCE):
            return q, position_norm, rotation_norm

        jacobian = robot.jacob0(q)
        regularized = jacobian @ jacobian.T + DAMPING**2 * np.eye(6)
        joint_step = jacobian.T @ np.linalg.solve(regularized, error)
        step_norm = np.linalg.norm(joint_step)
        if step_norm > MAX_JOINT_STEP:
            joint_step *= MAX_JOINT_STEP / step_norm

        candidate = q + joint_step
        if robot.qlim is not None:
            below_limits = candidate < robot.qlim[0] - 1e-9
            above_limits = candidate > robot.qlim[1] + 1e-9
            if np.any(below_limits | above_limits):
                joint_index = int(np.flatnonzero(below_limits | above_limits)[0])
                raise RuntimeError(
                    f"{label}: RMRC step would exceed AR4 joint "
                    f"{joint_index + 1} limit "
                    f"({candidate[joint_index]:.3f} rad; "
                    f"allowed {robot.qlim[0, joint_index]:.3f} to "
                    f"{robot.qlim[1, joint_index]:.3f} rad; "
                    f"position error {position_norm:.4f} m, "
                    f"orientation error {rotation_norm:.4f} rad)"
                )
        q = candidate

    error, _ = pose_error(robot, q, target_world)
    raise RuntimeError(
        f"{label}: RMRC did not converge "
        f"(position error {np.linalg.norm(error[:3]):.4f} m, "
        f"orientation error {np.linalg.norm(error[3:]):.4f} rad)"
    )


def rmrc_segment(robot, q_start, target_world, label, steps):
    """Generate joint samples while tracking a straight Cartesian pose path."""
    start_world = robot.fkine(q_start)
    references = rtb.ctraj(
        start_world, target_world, np.linspace(0.0, 1.0, steps + 1)
    )

    q = np.asarray(q_start, dtype=float).copy()
    samples = []
    maximum_position_error = 0.0
    maximum_rotation_error = 0.0
    for index, reference_world in enumerate(references):
        if index == 0:
            samples.append(q.copy())
            continue
        q, position_error, rotation_error = solve_pose_rmrc(
            robot, q, reference_world, f"{label}, sample {index}/{steps}"
        )
        samples.append(q.copy())
        maximum_position_error = max(maximum_position_error, position_error)
        maximum_rotation_error = max(maximum_rotation_error, rotation_error)

    return samples, maximum_position_error, maximum_rotation_error


def rmrc_branch_segment(robot, q_start, target_world, label, steps):
    """Track a Cartesian via-path selected from a valid IK branch."""
    ik_result = robot.ikine_LM(
        target_world,
        q0=q_start,
        joint_limits=True,
    )
    if not ik_result.success:
        raise RuntimeError(f"{label}: target IK failed: {ik_result.reason}")

    # The Cartesian via-poses keep the differential controller on a joint-limit-safe branch.
    joint_reference = rtb.jtraj(q_start, ik_result.q, steps + 1).q
    q = np.asarray(q_start, dtype=float).copy()
    samples = [q.copy()]
    maximum_position_error = 0.0
    maximum_rotation_error = 0.0
    for index, reference_q in enumerate(joint_reference[1:], start=1):
        reference_world = robot.fkine(reference_q)
        q, position_error, rotation_error = solve_pose_rmrc(
            robot, q, reference_world, f"{label}, sample {index}/{steps}"
        )
        samples.append(q.copy())
        maximum_position_error = max(maximum_position_error, position_error)
        maximum_rotation_error = max(maximum_rotation_error, rotation_error)

    return samples, maximum_position_error, maximum_rotation_error


def build_test_trajectory(steps_per_segment=100):
    robot = AR4_MK5()
    robot.q = np.zeros(robot.n)
    robot.base = SE3.Trans(3.75, 6.9, 0.15) * SE3.Rz(np.pi)

    mesh_paths = {
        "floor": PROJECT_ROOT / "Models" / "Floor" / "Floor.dae",
        "conveyor": PROJECT_ROOT / "Models" / "Conveyor" / "Conveyor.stl",
        "chassis": PROJECT_ROOT / "Models" / "Chassis" / "Chassis.stl",
        "wheel_FL": PROJECT_ROOT / "Models" / "Components" / "Wheel_1.stl",
        "comp_conv": PROJECT_ROOT / "Models" / "Components" / "Conveyor_Components.stl",
    }
    missing_paths = [path for path in mesh_paths.values() if not path.is_file()]
    if missing_paths:
        raise FileNotFoundError(
            "Missing simulation assets: " + ", ".join(str(path) for path in missing_paths)
        )

    meshes = {
        "floor": Mesh(
            filename=str(mesh_paths["floor"]),
            pose=SE3.Trans(2.7, 4.1, 0.01),
            scale=[0.04, 0.04, 1],
            collision=False,
        )
    }
    meshes.update(
        {
            name: Mesh(filename=str(path), pose=SE3.Trans(0, 0, 0), collision=False)
            for name, path in mesh_paths.items()
            if name != "floor"
        }
    )

    target_wheel = SE3.Trans(3.75, 7.45, 0.3)
    target_pickup = target_wheel * SE3.Rx(np.pi)
    target_approach = SE3.Trans(0, 0, 0.1) * target_pickup
    target_contact = SE3.Trans(0, 0, 0.04) * target_pickup
    target_lift = SE3.Trans(0, 0, 0.1) * target_contact

    component_target = SE3.Trans(3.15, 6.92, 0.4) * SE3.Ry(np.pi / 2)
    target_inserted = SE3.Trans(-0.065, 0, 0) * component_target

    q = robot.q.copy()
    trajectory = [q.copy()]
    reports = []
    attachment_index = None
    grasp_transform = None

    def add_segment(target, label):
        nonlocal q
        samples, position_error, rotation_error = rmrc_segment(
            robot, q, target, label, steps_per_segment
        )
        trajectory.extend(samples[1:])
        q = samples[-1]
        reports.append((label, position_error, rotation_error))

    def add_branch_segment(target, label):
        nonlocal q
        samples, position_error, rotation_error = rmrc_branch_segment(
            robot, q, target, label, steps_per_segment
        )
        trajectory.extend(samples[1:])
        q = samples[-1]
        reports.append((label, position_error, rotation_error))

    add_segment(target_approach, "approach")
    add_segment(target_contact, "contact")

    end_effector = robot.fkine(q)
    grasp_transform = end_effector.inv() * SE3(meshes["wheel_FL"].T)
    attachment_index = len(trajectory) - 1
    add_segment(target_lift, "lift")

    wheel_pose_after_lift = robot.fkine(q) * grasp_transform
    mesh_center = wheel_pose_after_lift.inv() * target_wheel
    target_mesh = target_inserted * mesh_center.inv()
    target_place = target_mesh * grasp_transform.inv()
    target_align = SE3.Trans(0.1, 0, 0) * target_place

    add_branch_segment(target_align, "align")
    add_segment(target_place, "place")

    trajectory = np.asarray(trajectory)
    if not np.isfinite(trajectory).all():
        raise RuntimeError("RMRC trajectory contains non-finite joint positions")
    if robot.qlim is not None and (
        np.any(trajectory < robot.qlim[0] - 1e-8)
        or np.any(trajectory > robot.qlim[1] + 1e-8)
    ):
        raise RuntimeError("RMRC trajectory exceeds AR4 joint limits")

    final_wheel_pose = robot.fkine(trajectory[-1]) * grasp_transform
    final_wheel_center = final_wheel_pose * mesh_center
    final_position_error = np.linalg.norm(final_wheel_center.t - target_inserted.t)
    final_angle, _ = tr2angvec(
        target_inserted.R @ final_wheel_center.R.T
    )
    if (final_position_error > POSITION_TOLERANCE
            or final_angle > ROTATION_TOLERANCE):
        raise RuntimeError(
            "Final wheel-center pose does not match the placement target "
            f"({final_position_error:.4f} m, {final_angle:.4f} rad error)"
        )
    reports.append(("wheel-center placement", final_position_error, final_angle))

    return robot, meshes, trajectory, attachment_index, grasp_transform, reports


def play_in_swift(robot, meshes, trajectory, attachment_index, grasp_transform):
    environment = swift.Swift()
    environment.launch(realtime=True)
    try:
        for mesh in meshes.values():
            environment.add(mesh)
        robot.q = trajectory[0]
        robot.add_to_env(environment)

        camera_target = robot.base.t + np.array([0, 0, 0.35])
        camera_position = robot.base.t + np.array([1.2, 1.0, 1.2])
        environment.set_camera_pose(camera_position, camera_target)

        print("Playing the AR4 RMRC pick-and-place test in Swift.")
        for index, q in enumerate(trajectory[1:], start=1):
            robot.q = q
            if index >= attachment_index:
                meshes["wheel_FL"].T = (
                    robot.fkine(q) * grasp_transform
                ).A
            environment.step(DT)

        print("Playback complete. Close the Swift window to exit.")
        environment.hold()
    finally:
        environment.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--steps-per-segment",
        type=int,
        default=100,
        help="number of Cartesian RMRC samples per state-1 segment",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="plan and validate the trajectory without launching Swift",
    )
    args = parser.parse_args()
    if args.steps_per_segment < 2:
        parser.error("--steps-per-segment must be at least 2")

    robot, meshes, trajectory, attachment_index, grasp_transform, reports = (
        build_test_trajectory(args.steps_per_segment)
    )
    print(f"Generated {len(trajectory)} RMRC joint samples for the AR4.")
    for label, position_error, rotation_error in reports:
        print(
            f"{label}: max sample error "
            f"{position_error * 1000:.3f} mm, "
            f"{np.degrees(rotation_error):.3f} deg"
        )
    print("All samples are finite and within the AR4 joint limits.")

    if not args.plan_only:
        play_in_swift(
            robot, meshes, trajectory, attachment_index, grasp_transform
        )


if __name__ == "__main__":
    main()
