#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "move_group_ros2/srv/move_to_pose.hpp"
using MoveToPoseSrv = move_group_ros2::srv::MoveToPose;


#include <std_srvs/srv/trigger.hpp>

class ReachToGraspNode : public rclcpp::Node
{
public:
  ReachToGraspNode()
  : Node("reach_to_grasp_service")
  {
    this->declare_parameter<std::string>("planning_group", "panda_arm");
    this->declare_parameter<std::string>("gripper_group", "panda_hand");

    std::string arm_group_name, gripper_group_name;
    this->get_parameter("planning_group", arm_group_name);
    this->get_parameter("gripper_group", gripper_group_name);

    RCLCPP_INFO(this->get_logger(), "ReachToGraspNode using arm group: %s, hand group: %s",
                arm_group_name.c_str(), gripper_group_name.c_str());

    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), arm_group_name);

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), gripper_group_name);

    // For Panda hand (two finger joints, mirrored with single value): directly set open/close values
    open_gripper_joint_values_.clear();
    close_gripper_joint_values_.clear();

    // Configure gripper open/close joint targets for Panda hand
    // const std::vector<std::string>& joint_names = gripper_group_->getJointNames();
    const auto& joint_names = gripper_group_->getJointNames();
    // std::size_t nj = joint_names.size();

    open_gripper_joint_values_.assign(joint_names.size(), 0.04);
    close_gripper_joint_values_.assign(joint_names.size(), 0.015);

    
    // // Make sure we have the right number of joints
    // // NOTE: replace with gripper_command action server - 40 for effort, position for 0.04

    service_ = this->create_service<MoveToPoseSrv>(
    "/reach_to_grasp",
    std::bind(&ReachToGraspNode::handle_request, this,
              std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "ReachToGrasp service '/reach_to_grasp' is ready.");
  }

private:
  // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Service<MoveToPoseSrv>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  std::vector<double> open_gripper_joint_values_;
  std::vector<double> close_gripper_joint_values_;


  geometry_msgs::msg::PoseStamped target_ps;

  void handle_request(const std::shared_ptr<MoveToPoseSrv::Request> req,
                      std::shared_ptr<MoveToPoseSrv::Response> res)
  {
    try
    {

      // Use the requested pose as grasp pose in base frame
      const geometry_msgs::msg::Pose& grasp_pose = req->target_pose;

      geometry_msgs::msg::PoseStamped grasp_ps;
      // grasp_ps.header.frame_id = "panda_link0";
      grasp_ps.header.frame_id = arm_group_->getPlanningFrame();
      grasp_ps.pose = grasp_pose;

      // 1) Open gripper
      if (!setGripper(open_gripper_joint_values_))
      {
        res->success = false;
        // res->message = "Failed to open gripper.";
        return;
      }

      // (Temp) Move down vertically in base frame (planning frame)
      if (!moveInBaseZ(grasp_ps, -0.05))
      {
        res->success = false;
        // res->message = "Failed to lift object.";
        return;
      }

      // 2) Move EEF along its own +Z axis by 0.01m
      if (!moveAlongEndEffectorZ(target_ps, 0.12))
      {
        res->success = false;
        // res->message = "Failed to move in base-Y.";
        return;
      }

      // 3) Close gripper
      if (!setGripper(close_gripper_joint_values_))
      {
        res->success = false;
        // res->message = "Failed to close gripper.";
        return;
      }

      // 4) Lift vertically in base frame (planning frame)
      if (!moveInBaseZ(target_ps, 0.1))
      {
        res->success = false;
        // res->message = "Failed to lift object.";
        return;
      }

      // 5) Open gripper to drop
      if (!setGripper(open_gripper_joint_values_))
      {
        res->success = false;
        // res->message = "Failed to reopen gripper.";
        return;
      }

      res->success = true;
      // res->message = "Grasp sequence executed successfully.";
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in grasp sequence: %s", e.what());
      res->success = false;
      // res->message = std::string("Exception: ") + e.what();
    }
  }

  bool setGripper(const std::vector<double>& joint_values)
  {

    // 1) Get current state with a small timeout
    moveit::core::RobotStatePtr current_state = gripper_group_->getCurrentState(1.0);

    if (current_state)
    {
      // Enforce bounds in case of tiny numerical violations
      current_state->enforceBounds();
      gripper_group_->setStartState(*current_state);
      RCLCPP_INFO(this->get_logger(),
                  "openGripper: Using current robot state as start state.");
    }
    else
    {
      // Fallback: construct a default state from the robot model
      RCLCPP_WARN(this->get_logger(),
                  "openGripper: No current robot state available within 1.0s, "
                  "falling back to default robot state.");

      moveit::core::RobotState default_state(gripper_group_->getRobotModel());
      default_state.setToDefaultValues();
      default_state.enforceBounds();
      gripper_group_->setStartState(default_state);
    }

    // 2) set execution tolerances for gripper closing
    gripper_group_->setPlanningTime(5.0);
    gripper_group_->setMaxVelocityScalingFactor(0.5);
    gripper_group_->setMaxAccelerationScalingFactor(0.5);  

    // 3) Set target joint values for the open posture
    gripper_group_->setJointValueTarget(joint_values);

    // 4) Plan and execute with time-out failure control to accomodate gripper closing on object
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = gripper_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Gripper planning failed.");
      return false;
    }
    auto exec = gripper_group_->execute(plan);
    if (exec == moveit::core::MoveItErrorCode::SUCCESS)
    {
      // RCLCPP_ERROR(this->get_logger(), "Gripper execution failed.");
      // return false;
      RCLCPP_INFO(this->get_logger(), "Gripper execution succeeded.");
      return true;
    }

    // 5) Change to accomodating gripper closing on objects
    if (exec == moveit::core::MoveItErrorCode::TIMED_OUT ||
        exec == moveit::core::MoveItErrorCode::CONTROL_FAILED)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Gripper: execution returned %d (TIMED_OUT / CONTROL_FAILED). "
                  "Assuming object contact and treating as success.",
                  static_cast<int>(exec.val));
      return true;
    }

    return false;
  }

  bool moveAlongEndEffectorZ(
      const geometry_msgs::msg::PoseStamped& ref_pose,
      double distance)
  {
      // geometry_msgs::msg::PoseStamped target = ref_pose;
      target_ps = ref_pose;

      // Compute world direction of EE Z axis
      tf2::Quaternion q;
      tf2::fromMsg(target_ps.pose.orientation, q);
      tf2::Matrix3x3 R(q);

      // Move along +Z_EE (or -Z_EE depending on how you set distance sign)
      tf2::Vector3 dz_ee(0.0, 0.0, distance);
      tf2::Vector3 dz_world = R * dz_ee;

      target_ps.pose.position.x += dz_world.x();
      target_ps.pose.position.y += dz_world.y();
      target_ps.pose.position.z += dz_world.z();

      arm_group_->setPoseTarget(target_ps);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto code = arm_group_->plan(plan);
      if (code != moveit::core::MoveItErrorCode::SUCCESS)
          return false;

      code = arm_group_->execute(plan);
      return code == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool moveInBaseZ(const geometry_msgs::msg::PoseStamped& ref_pose, double dz) // const std::shared_ptr<move_group_ros2::srv::MoveToPose::Request> req, 
  {
    const std::string ee_link = arm_group_->getEndEffectorLink();
    // geometry_msgs::msg::PoseStamped current = arm_group_->getCurrentPose(ee_link);

    // geometry_msgs::msg::Pose target = current.pose; // req->target_pose; // move_group_ros2

    target_ps = ref_pose;
    target_ps.pose.position.z += dz;  // base frame Z

    RCLCPP_INFO(this->get_logger(), "target_ps.pose.position.z = %f", target_ps.pose.position.z);

    arm_group_->setPoseTarget(target_ps, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = arm_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Planning base-Z lift failed.");
      return false;
    }

    auto exec = arm_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Execution of base-Z lift failed.");
      return false;
    }

    return true;
  }


  bool moveInBaseY(const geometry_msgs::msg::PoseStamped& ref_pose, double dy) // const std::shared_ptr<move_group_ros2::srv::MoveToPose::Request> req, 
  {
    const std::string ee_link = arm_group_->getEndEffectorLink();
    // geometry_msgs::msg::PoseStamped current = arm_group_->getCurrentPose(ee_link);

    // geometry_msgs::msg::Pose target = current.pose; // req->target_pose; // 
    // geometry_msgs::msg::PoseStamped target = ref_pose;
    target_ps = ref_pose;
    target_ps.pose.position.y += dy;  // base frame y

    RCLCPP_INFO(this->get_logger(), "target_ps.pose.position.y = %f", target_ps.pose.position.y);

    arm_group_->setPoseTarget(target_ps, ee_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = arm_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(this->get_logger(), "Planning base-Y reach failed.");
      return false;
    }

    auto exec = arm_group_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Execution of base-Y reach failed.");
      return false;
    }

    return true;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachToGraspNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
