# Move_Group_ROS2

This package provides ROS 2 **service servers and/or action servers** that wrap lower-level utilities
(e.g., PCL, perception, planning, or motion helpers) into reusable ROS 2 nodes.

These nodes are intended to be used as modular building blocks in larger systems such as
state machines, behavior trees, or task planners.

## ROS 2 Version

This package is built for **ROS 2** and has been **tested on ROS 2 Jazzy**.

## Package Overview

- Implements one or more **ROS 2 service servers and/or action servers**
- Designed to be called by higher-level coordination logic (FlexBE, SMACH, Behavior Trees, etc.)
- Focuses on wrapping non-ROS utilities into clean ROS 2 interfaces
- Intended for composition into larger manipulation or perception pipelines

## Requirements

This package depends on the following:

### System Dependencies
- Ubuntu 24.04 (tested)
- ROS 2 Jazzy

### ROS / Library Dependencies
```text
- rclcpp
- geometry_msgs
- moveit_ros_planning_interface
- moveit_msgs
- visualization_msgs
- sensor_msgs
- std_msgs
- tf2
- tf2_ros
- tf2_geometry_msgs
- tf2_sensor_msgs
```

## Important Conditional Dependency:
>It is **strongly recommended** that users follow the instructions at the
>[MoveIt Tutorials Setup Instructions](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
>page to **install MoveIt from source** **if you plan to use advanced utilities**
>such as the **MoveIt Task Constructor**.
>
>Make sure all dependencies are available in the ROS 2 workspace by following these instructions and then sourcing your new MoveIt workspace before building this >package and any other packages which rely on MoveIt or any of its libraries.

## Build

>If the MoveIt libraries and packages were installed from source following the instructions on the MoveIt Tutorials setup page:
```bash
source ~/ws_moveit/install/setup.bash
```

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select move_group_ros2
source install/setup.bash
```

## Usage

The nodes can be run directly, but rely on parameters such as the robot_description which are passed to the node and should be implemented into a launch file as demonstrated below:


**example.launch.py**
```python
def launch_setup(context, *args, **kwargs):
  robot_description = ...
  ...

  # example launch file implementation for move_to_pose_service node
  move_to_pose_service_node = Node(
        package="move_group_ros2",
        executable="move_to_pose_service",
        name="move_to_pose_service",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
        ],
    )

  return [
      move_to_pose_service_node,
  ]
```

>A full example launch file implementation can be found at [gazebo_move_group_flexbe.launch.py](https://github.com/uml-robotics/armada_ros2/blob/main/armada_bringup/launch/gazebo_move_group_flexbe.launch.py)

## Interfaces

### Services

| Service Name | Type | Description |
|-------------|------|-------------|
| `/move_to_pose_service` | `srv/MoveToPose` | Given a *string* **target_name**, move to the pose as defined in the robot's srdf with that name |
| `/move_to_named_pose_service` | `srv/MoveToNamedPose` | Move to the *geometry_msgs/PoseStamped* **target_pose** using the default planner (typically OMPL unless defined otherwise) |
| `/cartesian_move_to_pose_service` | `srv/CartesianMoveToPose` | Given *geometry_msgs/Pose[]* **waypoints**, move along a cartesian path to those waypoints (one or more) in order |
| `/reach_to_grasp_service` | `srv/MoveToPose` | A sequence of movements from pre-grasp pose to reach to grasp, pick up, and release object |

## Implementation

Below is an example implementation of a simple move_group_node service using the move_group C++ API

```cpp
#include ...

// create a class for your utility
class MoveGroupNode : public rclcpp::Node
{
public:
  MoveGroupNode()
    : Node("move_to_pose_service")
  {
    this->declare_parameter<std::string>("planning_group", "arm");

    std::string group_name;
    this->get_parameter("planning_group", group_name);

    // instantiate a move_group planning_interface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);

    // instantiate a service in your object constructor
    service_ = this->create_service<move_group_ros2::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&MoveGroupNode::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "MoveToPose service ready (group: %s)", group_name.c_str());
  }

private:
  // create shared class variables 
  rclcpp::Service<move_group_ros2::srv::MoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  void handle_request(
    const std::shared_ptr<move_group_ros2::srv::MoveToPose::Request> req,
    std::shared_ptr<move_group_ros2::srv::MoveToPose::Response> res)
  {
    try
    {
      // set up planning interface
      const std::string planning_frame = move_group_->getPlanningFrame();
      move_group_->setPoseReferenceFrame(planning_frame);

      // tell the robot where it is and where it is going
      move_group_->setStartStateToCurrentState();
      move_group_->setPoseTarget(req->target_pose);

      // set planning parameters and plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group_->setPlanningTime(3.0);
      move_group_->setGoalTolerance(1e-3);
      move_group_->setGoalOrientationTolerance(0.15);
      auto planning_result = move_group_->plan(plan);
  
      // execute plan or fail depending on planning result, report result
      if (planning_result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_WARN(this->get_logger(), "Planning failed.");
        res->success = false;
        return;
      }
  
      RCLCPP_INFO(this->get_logger(), "Planning succeeded. Executing...");
  
      moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);
  
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Motion to pose succeeded.");
        res->success = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Motion execution failed.");
        res->success = false;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during planning or execution: %s", e.what());
      res->success = false;
    }
  }
};

int main(int argc, char **argv)
{
  // initialize and spin your node in the background
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveGroupNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

> This is a slightly stripped-down version of the move_to_pose_service node which exists in this package under src/, please check out that node for more information, or visit the [MoveIt Documentation](https://moveit.picknik.ai/main/doc/concepts/move_group.html) for more information regarding the move_group and its implementation/usage