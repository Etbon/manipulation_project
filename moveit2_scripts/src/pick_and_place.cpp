#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cstdio>
#include <string>
#include <vector>
#include <memory>
#include <thread>

class PickAndPlace : public rclcpp::Node {
  public:
    explicit PickAndPlace()
      : Node("pick_and_place_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

      RCLCPP_INFO(get_logger(), "Pick and place node is ready...");
    }

    // Creat the interface that we will use to request robot motion
    bool initialize_moveit() {
      arm_ = std::make_unique<MoveGroupInterface>(shared_from_this(),"ur_manipulator");
      gripper_ = std::make_unique<MoveGroupInterface>(shared_from_this(), "gripper");

      // Print Info
      RCLCPP_INFO(get_logger(), "Planning group: %s", arm_->getName().c_str());
      RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());    

      // Current joint state Arm & Gripper
      if (!arm_->getCurrentState(10.0) || !gripper_->getCurrentState(10.0)) {
        RCLCPP_ERROR(get_logger(), "Current robot state unavailable.");
        return false;
      }

      // Slow motion is useful while validating the simulation setup 
      arm_->setMaxVelocityScalingFactor(0.10);
      arm_->setMaxAccelerationScalingFactor(0.10);

      gripper_->setMaxVelocityScalingFactor(0.10);
      gripper_->setMaxAccelerationScalingFactor(0.10);

      tool_link_ = arm_->getEndEffectorLink();
      const auto *gripper_model = gripper_->getRobotModel()->getJointModelGroup("gripper");
      
      if (tool_link_.empty() || !gripper_model) {
        RCLCPP_ERROR(get_logger(), "Check the end-effector and gripper configuration.");
        return false;
      }

      touch_links_ = gripper_model->getLinkModelNames();

      return true;
    } 
    
    // -----------------
    // Arm movement logic 
    // -----------------
    bool run_pick_and_place() {

      // Remeber the starting joint postion 
      const auto initial_state = arm_->getCurrentState(2.0);

      if (!initial_state) {
        RCLCPP_ERROR(get_logger(), "Cannot read the starting robot state.");
        return false;
      }

      // Call make_pregrasp_pose() and store its retrun pose
      const auto pregraspe_pose = make_pregrasp_pose();
      
      // Create a Plan object named pregrasp_plan
      Plan pregraspe_plan;

      // Planning and execution safty check
      if (!plan_arm_motion(pregraspe_pose, pregraspe_plan)) return false;       
      RCLCPP_INFO(get_logger(), "Plan calculated.");
    
      // Execute the trajectory 
      if (!execute_arm_motion(pregraspe_plan)) return false;    
      RCLCPP_INFO(get_logger(), "Executed.");

      // Creat a Plan object for the gripper 
      Plan open_plan;
      // Plan gripper 
      if (!plan_gripper_name_motion("gripper_open", open_plan)) return false;
      RCLCPP_INFO(get_logger(), "Plan gripper.");
      // Execute gripper
      if (!execute_gripper_motion(open_plan)) return false;
      RCLCPP_INFO(get_logger(), "Execute open gripper.");

      // Create a target 9 cm below the pregrasp pose
      const auto grasp_pose = make_grasp_pose(pregraspe_pose); 
      
      Plan approach_plan;
      // Decent toward the block 
      if(!plan_cartesian_motion(grasp_pose, approach_plan)) return false;
      // Ececute arm motion
      if (!execute_arm_motion(approach_plan)) return false;

      // Close gripper 
      Plan close_plan;
      // Plan gripper 
      if (!plan_gripper_joint_motion(kCloseAngle, close_plan)) return false;
      RCLCPP_INFO(get_logger(),"Plan gripper...");
      // Execute gripper
      if (!execute_gripper_motion(close_plan)) return false;
      RCLCPP_INFO(get_logger(), "Execute close gripper.");

      // Lift object up 
      Plan lift_object;
      if(!plan_cartesian_motion(pregraspe_pose, lift_object)) return false;
      RCLCPP_INFO(get_logger(), "Plan lift object...");

      if(!execute_arm_motion(lift_object)) return false;
      RCLCPP_INFO(get_logger(), "Execute lift object.");

      // Move to postion 180 degees 
      const auto current_state = arm_->getCurrentState(2.0);  // get current arm state  

      // Safety gaurd  
      if (!current_state) {
        RCLCPP_ERROR(get_logger(), "Cannot read robot state before truning.");
        return false;
      }      
      
      // Current state 
      moveit::core::RobotState turn_target = *current_state;
     
      // Current angle
      const double shoulder_angle = turn_target.getVariablePosition("shoulder_pan_joint");

      // Half turn
      turn_target.setVariablePosition("shoulder_pan_joint", shoulder_angle + M_PI);

      Plan turn_plan; 

      // Calculate the trajectory and store it in turn_plan
      if (!plan_arm_joint_motion(turn_target, turn_plan)) return false;
      
      // Execute the trajcetory in trun_plan
      if (!execute_arm_motion(turn_plan)) return false;

      // Open gripper
      Plan realese_plan;
      if (!plan_gripper_name_motion("gripper_open", realese_plan)) return false;
      if (!execute_gripper_motion(realese_plan)) return false;


      // Move to the original position
      Plan return_plan;
      if (!plan_arm_joint_motion(*initial_state, return_plan)) return false;
      if (!execute_arm_motion(return_plan)) return false;  
    
      return true;
    }

  private:
    // using shorthand for lenghty class references
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface; 
    using JointModelGroup = moveit::core::JointModelGroup;
    using RobotStatePtr = moveit::core::RobotStatePtr;
    using Plan = MoveGroupInterface::Plan;
    using Pose = geometry_msgs::msg::Pose;

    static constexpr double kSpeedScale = 0.10;   
    static constexpr double kApproachDistance = 0.05;  // metres requeted 5 cm 
    static constexpr double kCloseAngle = 0.64;       // radians
    
    std::unique_ptr<MoveGroupInterface> arm_;
    std::unique_ptr<MoveGroupInterface> gripper_;
    std::string tool_link_;
    std::vector<std::string> touch_links_;

    std::vector<double> make_home_joint_target() {
      return {
        0.0000,    // Shoulder pan
        -2.3562,   // Shoulder lift
        1.5708,    // Elbow
        -1.5708,   // Wrist 1
        -1.5708,   // Wrist 2
        0.0000,    // Wrist 3
      };    
    }

    geometry_msgs::msg::PoseStamped make_pregrasp_pose() {
      geometry_msgs::msg::PoseStamped target;

      // These coordinates are measured from the robot base.
      target.header.frame_id = "base_link";

      // Align horizontally with the block. Sim & Real
      target.pose.position.x =  this->get_parameter_or<double>("pick_x", 0.339984);
      target.pose.position.y = this->get_parameter_or<double>("pick_y", -0.020271);

      // Candidate pregrasp TOOL height from the original example.
      // This still needs checking against the fingertip position.
      target.pose.position.z = 0.264;

      // Desired tool orientation relative to base_link.
      target.pose.orientation.x = -1.0;
      target.pose.orientation.y =  0.0;
      target.pose.orientation.z =  0.0;
      target.pose.orientation.w =  0.0;

      RCLCPP_INFO(
        get_logger(),
        "Pregrasp in %s: x=%.3f, y=%.3f, z=%.3f",
        target.header.frame_id.c_str(),
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z
      );

      return target;
    }

    geometry_msgs::msg::PoseStamped make_grasp_pose(const geometry_msgs::msg::PoseStamped &pregrasp) {

      auto grasp_pose = pregrasp;
      grasp_pose.pose.position.z -= 0.09;

      return grasp_pose;
    }

    // ---------
    // Plan Arm 
    // ---------
    bool plan_arm_motion(const geometry_msgs::msg::PoseStamped &target_pose, Plan &plan) {

      // Remove tagets left by an earlier request 
      arm_->clearPoseTargets();

      // Plan from the robot's current configuration
      arm_->setStartStateToCurrentState();

      // Choos the desired tool destination
      if (!arm_->setPoseTarget(target_pose, tool_link_)) {
        RCLCPP_ERROR(get_logger(), "Could not set the arm target.");
        arm_->clearPoseTargets();
        return false;
      }

      // Calculate a trajectory and store it in the supplied plan 
      const auto result = arm_->plan(plan);

      // The calculated trajectory remains stored in 'plan'
      arm_->clearPoseTargets();

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Arm planning failed.");
        return false;
      }

      RCLCPP_INFO(get_logger(), "Arm plan calculated successfully");
      return true;
    }

    // ---------------
    // Plan Arm Joints
    // ---------------
    bool plan_arm_joint_motion(const moveit::core::RobotState &target_stat, Plan &plan) {
      // Remove tool-pose goals left by pevious planning requests
      arm_->clearPoseTargets();

      // Start this plan from the robot's current joint configuration
      arm_->setStartStateToCurrentState();
    
      // Set the desired final joint configuration 
      if (!arm_->setJointValueTarget(target_stat)) {
        RCLCPP_ERROR(get_logger(), "Arm joint target is outside its limits.");
        return false;
      }

      // Calculate a trajectory and store it in the supplied plan
      const auto result = arm_->plan(plan);

      // Stop if MoveIt could not calculate a successful plan
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Arm joint planning failed.");
        return false;
      }

      // Planning succeeded
      return true;
    }
    
    // --------------------
    //  Plan Arm Cartisian 
    // --------------------
    bool plan_cartesian_motion(const geometry_msgs::msg::PoseStamped &target_pose, Plan &plan) {
      // Get the current state of the arm   
      const auto start = arm_->getCurrentState(2.0);

      // Safty gaurd
      if (!start) {
        RCLCPP_ERROR(get_logger(), "Current start unavailabel for Cartisian planning");
        return false;
      }
      
      // Start planning from the robot's current joint configuration
      arm_->setStartState(*start);

      // Measure waypoint coordinates relative to this frame
      arm_->setPoseReferenceFrame(target_pose.header.frame_id);

      // Creat an empty list of poses
      std::vector<Pose> waypoint;
      waypoint.push_back(target_pose.pose);  // Add the desired tool position and orientation

      const double eef_step = 0.005;   // Maximum spacing between samples tool posiotion 5mm 
      const double jump_threshold = 2.0; // Detect joint-space steps larger than twice the average step

      // Calculate the Cartisain trajectory and store the compleation fraction
      const double fraction = arm_->computeCartesianPath(
        waypoint, 
        eef_step, 
        jump_threshold, 
        plan.trajectory_, 
        true
      );

      // Check wether the entire requested path was calulated
      if (fraction != 1.0) {
        RCLCPP_ERROR(get_logger(), "Cartesian palnning failed. Fraction: %.3f", fraction);
        return false;
      }
  
      return true;

    }
    
    // ------------
    // Execute Arm
    // ------------
    bool execute_arm_motion(const Plan &plan) {

      // Execute the supplied plan and store the returned status
      const auto result = arm_->execute(plan);

      // If the status is not SUCCESS: 
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Arm execute failed");
        return false;
      }

      RCLCPP_INFO(get_logger(), "Arm executed successfully ");
      return true;
    }

    // -------------
    // Plan Gripper 
    // -------------
    bool plan_gripper_name_motion(const std::string &target_name, Plan &plan) {
      // Start planning from the gripper's current positon
      gripper_->setStartStateToCurrentState();

      // Gaurd if unknow gripper target
      if (!gripper_->setNamedTarget(target_name)) {
        RCLCPP_ERROR(get_logger(), "Unknow gripper target: %s",
          target_name.c_str());
        return false; 
      }

      // Calculate the trajecotry and store it in the caller's plan
      const auto result = gripper_->plan(plan);

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Gripper plannig failed.");
        return false;
      }

      return true;
    }

    bool plan_gripper_joint_motion(double angle, Plan &plan) {
      // Start from last postion 
      gripper_->setStartStateToCurrentState();
      
      // Choose the finger joint's target angel, in radians
      if (!gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", angle)) {
        RCLCPP_ERROR(get_logger(), "Invalid grpper target: %.3f rad", angle);
        return false;
      }

      // Caclulate the trajectory and store it in plan
      const auto result = gripper_->plan(plan);

      // Report failur if no trajectroy could be calculated
      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Gripper planning failed.");
        return false;
      }

      // Trajectory successful
      return true;
    }

    // ----------------
    // Execute Gripper
    // ----------------
    bool execute_gripper_motion(const Plan &plan) {

      const auto result = gripper_->execute(plan);

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Gripper execution failed");
        return false;
      }

      return true;
    }

};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); // Initialize ROS using command-line arguments
  auto node = std::make_shared<PickAndPlace>(); // Creat ROS node

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); // Register the node's callback with the executor
  std::thread spinner([&executor]() { executor.spin(); });
  
  const bool moveit_ready = node->initialize_moveit();

  if (moveit_ready) {
    RCLCPP_INFO(node->get_logger(), "MoveIt initilaize succeeded.");
    node->run_pick_and_place();
  }

  rclcpp::shutdown();
  spinner.join();  
  return 0;
}
