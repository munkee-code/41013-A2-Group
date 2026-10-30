#!/usr/bin/env python3
import time
import roslibpy
import numpy as np
import roboticstoolbox as rtb

current_pos = None
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
SQUARE_WAYPOINTS = np.array([
    [0.00, 0.00, 0.02],
    [0.06, 0.00, 0.02],
    [0.06, 0.00, 0.08],
    [0.00, 0.00, 0.08],
    [0.00, 0.00, 0.02],
])
STEPS_PER_SIDE = 25

def joint_state_cb(message):
    global current_pos
    names = message.get('name', [])
    positions = message.get('position', [])
    if len(names) != len(positions):
        raise ValueError("JointState name and position arrays have different lengths")
    if any(name not in names for name in JOINT_NAMES):
        raise ValueError(
            f"JointState is missing one or more UR3 joints: {names}"
        )
    position_by_name = dict(zip(names, positions))
    current_pos = [float(position_by_name[name]) for name in JOINT_NAMES]

def square_traj(robot, q_start, waypoints=SQUARE_WAYPOINTS, steps_per_side=STEPS_PER_SIDE):
    """Convert fixed Cartesian offsets from the current pose into joint waypoints."""
    q = np.array(q_start, dtype=float)
    start_pose = robot.fkine(q)
    trajectory = [q.copy()]

    for offset in waypoints:
        target_pose = start_pose.A.copy()
        target_pose[:3, 3] += np.asarray(offset, dtype=float)
        result = robot.ikine_LM(
            target_pose,
            q0=q,
            mask=[1, 1, 1, 0, 0, 0],
            joint_limits=True,
        )
        if not result.success:
            raise RuntimeError(
                f"Square waypoint {np.asarray(offset).tolist()} is unreachable: "
                f"{result.reason}"
            )

        segment = rtb.jtraj(q, result.q, steps_per_side).q
        trajectory.extend(segment[1:])
        q = np.asarray(result.q, dtype=float)

    return trajectory

def move_ur_joint_positions(client, joint_positions, duration=5.0):
    global current_pos
    
    try:
        client.run()

        # Subscribe to joint states to get the current position
        listener = roslibpy.Topic(client, '/ur/joint_states', 'sensor_msgs/JointState')
        listener.subscribe(joint_state_cb)

        # Wait until we receive a joint state
        print("[ROS] Waiting for current joint state...")
        start_time = time.time()
        while current_pos is None and time.time() - start_time < 5.0:
            time.sleep(0.05)
        if current_pos is None:
            raise RuntimeError("No joint state received from /ur/joint_states")

        print(f"[ROS] Current joint positions: {current_pos}")

        joint_positions_array = np.asarray(joint_positions, dtype=float)
        if joint_positions_array.ndim == 1:
            joint_positions_array = joint_positions_array.reshape(1, -1)
        if joint_positions_array.shape[1] != 6:
            raise ValueError(
                f"Expected joint positions with shape (N, 6), got "
                f"{joint_positions_array.shape}"
            )

        points = [
            {
                'positions': current_pos,
                'time_from_start': {'secs': 0, 'nsecs': 0}
            }
        ]

        for i, q in enumerate(joint_positions_array):
            t = duration * (i + 1) / len(joint_positions_array)
            secs = int(t)
            nsecs = int(round((t - secs) * 1e9))

            if nsecs >= 1e9:
                secs += 1
                nsecs -= int(1e9)
            points.append({
                'positions': q.tolist(),
                'time_from_start': {'secs': secs, 'nsecs': nsecs}
            })

        trajectory_msg = {
            'joint_names': JOINT_NAMES,
            'points': points
        }

        # Publish to the controller's /command topic
        topic = roslibpy.Topic(
            client,
            '/ur/scaled_pos_joint_traj_controller/command',
            'trajectory_msgs/JointTrajectory'
        )
        topic.advertise()
        topic.publish(roslibpy.Message(trajectory_msg))
        print("[ROS] Trajectory published.")

        # Wait for motion to complete
        time.sleep(duration + 1.0)

        topic.unadvertise()
        listener.unsubscribe()

    finally:
        print("[ROS] Finish")

if __name__ == '__main__':
    client = roslibpy.Ros(host='192.168.27.1', port=9090)  # Replace with your ROS bridge IP
    client.run()

    try:
        # NEW: Read current joint positions for RMRC
        listener = roslibpy.Topic(
            client,
            '/ur/joint_states',
            'sensor_msgs/JointState'
        )
        listener.subscribe(joint_state_cb)

        start_time = time.time()
        while current_pos is None and time.time() - start_time < 5.0:
            time.sleep(0.05)

        if current_pos is None:
            raise RuntimeError("No joint state received")

        listener.unsubscribe()

        # Create UR3 model
        robot = rtb.models.DH.UR3()

        # JointState positions are ordered by JOINT_NAMES in joint_state_cb.
        q_start = current_pos.copy()

        # Follow the hard-coded square corners relative to the current pose.
        trajectory = square_traj(
            robot,
            q_start,
        )

        print(f"[ROS] Generated {len(trajectory)} joint waypoints")
        print("[ROS] Sending hard-coded square trajectory to controller")

        move_ur_joint_positions(
            client,
            trajectory,
            duration=12.0
        )
    except KeyboardInterrupt:
        print("\n[APP] Interrupted by user.")
