from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
  # Load the configuratino for our robot
  moveit_config = (
    MoveItConfigsBuilder("ur", package_name="my_moveit_config").to_moveit_configs()
  )

  # Start our compiled C++ executable with its parameters
  motion_node = Node(
    package="moveit2_scripts",
    executable="pick_and_place_node",
    name="pick_and_place_node",
    output="screen",
    parameters=[
      moveit_config.robot_description,
      moveit_config.robot_description_semantic,
      moveit_config.robot_description_kinematics,
      {"use_sim_time" : True},
    ],
  )

  # Not in use 
  perception_node = Node(
    package="simple_grasping",
    executable="basic_grasping_perception_node",
    output="screen",
    parameters=[
      {"debug_topics": True, "use_sim_time": True},
    ],
  )


  return LaunchDescription([
    motion_node,
    perception_node
  ])