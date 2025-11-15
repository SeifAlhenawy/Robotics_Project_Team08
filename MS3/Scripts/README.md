This folder contains all the Python scripts used for validating **Forward / Inverse Position** and **Forward / Inverse Velocity** Kinematics on the simulated 5-DOF robotic arm inside **Gazebo** using ROS1 Noetic.

All scripts read parameters from the ROS parameter server and compare the results with the Gazebo end-effector pose to ensure correctness.

---

## 📂 Folder Contents

| File Name | Description |
|-----------|------------|
| `joint_publisher_param.py` | Publishes joint angles to Gazebo using ROS parameters (`rosparam set Joint_i`) in radians. |
| `ee_listener.py` | Subscribes to `/gazebo/link_states` and prints the real end-effector position (x,y,z) in Gazebo. |
| `fk_from_params_node.py` | Computes Forward Position Kinematics and prints predicted EE (x,y,z). |
| `ik_from_task_node.py` | Solves Inverse Position Kinematics using Newton-Raphson. |
| `vel_forward_node.py` | Computes Forward Velocity Kinematics using Jacobian. |
| `vel_inverse_node.py` | Computes Inverse Velocity Kinematics using pseudo-inverse of Jacobian. |
| `ee_pose_from_gazebo.py` | Reads EE Pose from `/gazebo/model_states`. |
| `show_urdf_path.py` | Displays URDF path. |
| `joint_publisher_fixed.py` | Fixed-angle test file. |
| `joint_publisher.py` | Basic joint publisher example. |
| `kinematic_joint_driver.py` | Driver for future real-time control. |

---

## 🧪 Testing Commands

### Forward Position:
```bash
rosrun my_robot_gazebo joint_publisher_param.py
rosparam set Joint_1 30
rosparam set Joint_2 30
rosparam set Joint_3 -30
rosparam set Joint_5 30
rosrun my_robot_gazebo fk_from_params_node.py

Inverse Position:
rosparam set IK_X_des 0.10
rosparam set IK_Y_des 0.05
rosparam set IK_Z_des 0.20
rosrun my_robot_gazebo ik_from_task_node.py


Forward Velocity:
rosparam set J1_dot 1.0
rosparam set J2_dot 1.0
rosparam set J3_dot 1.0
rosparam set J5_dot 1.0
rosrun my_robot_gazebo vel_forward_node.py


Inverse Velocity:
rosparam set X_dot 0.1
rosparam set Y_dot 0.1
rosparam set Z_dot 0.05
rosrun my_robot_gazebo vel_inverse_node.py
