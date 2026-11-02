# UR3e Pick-and-Place with ROS 2 and MoveIt 2

A C++ manipulation project for a **Universal Robots UR3e with a Robotiq 85 gripper**, using ROS 2 Humble and MoveIt 2. Separate configurations connect the same application to a simulation environment or the real robot.

The task moves through a joint-space waypoint, makes a Cartesian adjustment above the object, opens the gripper, approaches, closes, lifts, turns, releases, and returns home.

This repository contains the application and MoveIt configurations. The compatible robot description, simulator, hardware drivers, and gripper action server must be provided by the surrounding lab environment.

## Packages

| Package | Purpose |
| --- | --- |
| [`moveit2_scripts`](moveit2_scripts) | C++ pick-and-place application and task launch files. |
| [`my_moveit_config`](my_moveit_config) | MoveIt configuration for simulation. |
| [`real_moveit_config`](real_moveit_config) | MoveIt configuration for the physical robot. |

Main application: [`moveit2_scripts/src/pick_and_place.cpp`](moveit2_scripts/src/pick_and_place.cpp).

## Requirements

- An existing ROS 2 Humble environment with MoveIt 2, `colcon`, and `rosdep`.
- A C++17 compiler and the ROS dependencies declared in the packages.
- A compatible `ur_description` setup containing the arm and gripper joints expected by these configurations.
- For simulation: the lab simulator, robot controllers, joint states, transforms, and simulation clock.
- For hardware: the UR robot driver, a running robot control program, and the Robotiq gripper action server.
- The `simple_grasping` package for the current simulation task launch. That launch starts its perception node, although the C++ task does not consume its detections.

The repository does not include a simulator world or a complete hardware bringup launch. Installing the declared dependencies alone may not reproduce the original lab environment.

## Build

Clone the repository into a ROS workspace. Skip the clone command if the repository is already present.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Etbon/manipulation_project.git

cd ~/ros2_ws
rosdep install --from-paths src/manipulation_project --ignore-src -r -y --rosdistro humble
colcon build --packages-select moveit2_scripts my_moveit_config real_moveit_config --symlink-install
source install/setup.bash
```

`manipulation_project` is the repository directory. The three names passed to `--packages-select` are the actual ROS packages.

In every new terminal used below, load the environment first:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Run in simulation

1. Start the compatible lab simulator and its arm and gripper controllers.
2. Start MoveIt if the lab has not already started it:

   ```bash
   ros2 launch my_moveit_config move_group.launch.py
   ```

3. Optionally open RViz in another terminal:

   ```bash
   ros2 launch my_moveit_config moveit_rviz.launch.py
   ```

4. Start the application in another terminal:

   ```bash
   ros2 launch moveit2_scripts pick_and_place.launch.py
   ```

The simulation task uses `use_sim_time: true`, so its clock must be running. The task launch starts the application and the `simple_grasping` perception node; it does not start the simulator or MoveIt server.

## Run on the real robot

The task starts moving automatically after initialization. Before launching it, validate the stored joint configurations, tool geometry, target heights, and planning scene for the actual setup. Review the planned paths and use the lab's supervised commissioning procedure.

1. Start the lab's UR driver and gripper server, and ensure the robot control program is running.
2. Start MoveIt if a correctly configured instance is not already running:

   ```bash
   ros2 launch real_moveit_config move_group.launch.py
   ```

3. Optionally open RViz in another terminal:

   ```bash
   ros2 launch real_moveit_config moveit_rviz.launch.py
   ```

4. Start the application in another terminal:

   ```bash
   ros2 launch moveit2_scripts pick_and_place_real.launch.py
   ```

The real-robot task uses `use_sim_time: false`. Reuse the lab's MoveIt instance when one is already running; launching a second instance can create conflicting action servers and configuration.

## How the task works

| Stage | Command | Purpose |
| --- | --- | --- |
| 1 | Joint plan to the stored home configuration | Establish the chosen starting configuration. |
| 2 | Joint plan to the intermediate waypoint | Approach the pick area with a chosen arm posture. |
| 3 | Cartesian adjustment to the pregrasp pose | Align the selected tool link above the object. |
| 4 | Gripper plan to `gripper_open` | Open the fingers before descending. |
| 5 | Cartesian approach | Lower the tool by the configured approach distance. |
| 6 | Gripper joint plan | Close the gripper to the configured joint target. |
| 7 | Cartesian lift | Return to the pregrasp height. |
| 8 | Joint plan with shoulder pan increased by pi radians | Move toward the release area. |
| 9 | Gripper plan to `gripper_open` | Release the object. |
| 10 | Joint plan to the stored home configuration | Finish at the configured home position. |

The final home target is a fixed configuration in the code, rather than the joint positions measured when the program started.

Each planning helper creates a trajectory. Its execution helper then waits for MoveIt to report the execution result. A failed helper ends the sequence immediately.

### Why combine a joint waypoint and Cartesian motion?

A joint target specifies the arm's configuration. A tool pose specifies a position and orientation, which may have several inverse-kinematics solutions. Planning directly to a tool pose can therefore produce an unexpected change in arm posture.

The intermediate joint waypoint selects a posture near the pick area. The following Cartesian adjustment takes the tool to the precise pregrasp pose. This can reduce unwanted configuration changes, but the complete path still needs collision checking and validation against the real setup.

The stored waypoint is defined in `pre_grasp_position()`, and the home configuration is defined in `make_home_joint_target()`. Their vectors must follow the joint order in the `ur_manipulator` planning group. Do not copy values directly from a `/joint_states` message without matching each value to its joint name.

## Targets and settings

The following values describe the current source. Distances are in metres and joint angles are in radians.

| Setting | Current value | Location |
| --- | --- | --- |
| Arm planning group | `ur_manipulator` | `initialize_moveit()` |
| Gripper planning group | `gripper` | `initialize_moveit()` |
| Cartesian target frame | `base_link` | `make_pregrasp_pose()` |
| Default pick X | `0.339984` | `pick_x` ROS parameter fallback |
| Default pick Y | `-0.020271` | `pick_y` ROS parameter fallback |
| Real launch pick X / Y | `0.343` / `0.132` | Real task launch parameters |
| Pregrasp tool height | `0.264` | `make_pregrasp_pose()` |
| Tool orientation, quaternion X/Y/Z/W | `-1 / 0 / 0 / 0` | `make_pregrasp_pose()` |
| Approach distance | `0.07` | `kApproachDistance` |
| Resulting grasp tool height | `0.194` | Pregrasp height minus approach distance |
| Gripper close target | `0.70` | `kCloseAngle` |
| Controlled gripper joint | `robotiq_85_left_knuckle_joint` | Gripper planning helper |
| Cartesian sampling step | `0.005` | `plan_cartesian_motion()` |
| Cartesian jump threshold | `2.0` | Relative joint-jump factor, not an angle |

Some source comments still mention a 9 cm approach; the current constant requests **7 cm**. The target height refers to the selected end-effector link, obtained through `getEndEffectorLink()`, rather than directly to the fingertips. Confirm that link and its offset before changing heights.

To change the real pick coordinates, edit the parameter dictionary in [`pick_and_place_real.launch.py`](moveit2_scripts/launch/pick_and_place_real.launch.py). The current launch file does not declare `pick_x` or `pick_y` as launch arguments. Other settings in the table are currently changed in C++; rebuild `moveit2_scripts` after editing them.

`kSpeedScale` is declared as `0.10`, but the application does not apply it through MoveIt's velocity or acceleration scaling setters. Its presence does not establish a 10% motion speed.

## Controller connections

MoveIt sends arm trajectories through a `FollowJointTrajectory` action and gripper commands through a `GripperCommand` action.

| Environment | Arm action | Gripper action |
| --- | --- | --- |
| Simulation | `/joint_trajectory_controller/follow_joint_trajectory` | `/gripper_controller/gripper_cmd` |
| Real robot | `/scaled_joint_trajectory_controller/follow_joint_trajectory` | `/robotiq_gripper_controller/gripper_cmd` |

These mappings come from the respective [`simulation`](my_moveit_config/config/moveit_controllers.yaml) and [`real-robot`](real_moveit_config/config/moveit_controllers.yaml) controller configuration files. The corresponding action servers must already exist.

The real configuration disables `trajectory_execution.execution_duration_monitoring`. The application also has no independent execution deadline. Consequently, an accepted trajectory can leave a blocking execution call waiting if the controller stops progressing without reporting a terminal result.

https://github.com/user-attachments/assets/c95bfd7f-7944-4dad-a644-0cccf02bb492

## References

- [MoveIt 2 Humble tutorials](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)
- [MoveGroupInterface API](https://moveit.picknik.ai/humble/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html)
- [Universal Robots ROS 2 controllers](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_controllers/doc/index.html)
- [Understanding protective stops](https://www.universal-robots.com/articles/ur/robot-care-maintenance/understanding-protective-stops/)

