from launch import LaunchDescription
from launch_ros.actions import Node 
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
  # Load the configuration for our robot 
  moveit_config = (MoveItConfigsBuilder("ur", package_name="real_moveit_config").to_moveit_configs())

  # Executabel with it's parameters
  motion_node = Node(
    package="moveit2_scripts",
    executable="pick_and_place_node",
    name="pick_and_place_node",
    output="screen",
    parameters=[
      moveit_config.robot_description,
      moveit_config.robot_description_semantic,
      moveit_config.robot_description_kinematics,
      {
        "use_sim_time" : False,
        "pick_x": 0.343,
        "pick_y": 0.132,
      },
    ],
  )

  return LaunchDescription([
    motion_node
  ])
