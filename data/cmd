ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0}, angular:{z: 0}}"

ros2 topic pub -1 /set_joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {frame_id: 'arm_base_link'}, joint_names: ['base_forearm_joint', 'forearm_hand_joint'], points: [{positions: [0.0, 0.0]}]}"
