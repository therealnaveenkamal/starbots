#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class GetPoseClient : public rclcpp::Node {
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit GetPoseClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("pick_and_place_perception_sim", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Find>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "find_objects");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&GetPoseClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GetPoseClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&GetPoseClient::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFind::SharedPtr,
                         const std::shared_ptr<const Find::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
  }

  void result_callback(const GoalHandleFind::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      rclcpp::NodeOptions node_options1;
      node_options1.automatically_declare_parameters_from_overrides(true);
      auto move_group_node = rclcpp::Node::make_shared(
          "move_group_interface_tutorial", node_options1);

      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(move_group_node);
      std::thread([&executor]() { executor.spin(); }).detach();

      static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
      static const std::string PLANNING_GROUP_GRIPPER = "gripper";
      moveit::planning_interface::MoveGroupInterface move_group_arm(
          move_group_node, PLANNING_GROUP_ARM);
      moveit::planning_interface::MoveGroupInterface move_group_gripper(
          move_group_node, PLANNING_GROUP_GRIPPER);
      const moveit::core::JointModelGroup *joint_model_group_arm =
          move_group_arm.getCurrentState()->getJointModelGroup(
              PLANNING_GROUP_ARM);
      const moveit::core::JointModelGroup *joint_model_group_gripper =
          move_group_gripper.getCurrentState()->getJointModelGroup(
              PLANNING_GROUP_GRIPPER);

      // Get Current State
      moveit::core::RobotStatePtr current_state_arm = move_group_arm.getCurrentState(10);
      moveit::core::RobotStatePtr current_state_gripper = move_group_gripper.getCurrentState(10);
      std::vector<double> joint_group_positions_arm;
      std::vector<double> joint_group_positions_gripper;
      current_state_arm->copyJointGroupPositions(joint_model_group_arm, joint_group_positions_arm);
      current_state_gripper->copyJointGroupPositions(joint_model_group_gripper, joint_group_positions_gripper);
      move_group_arm.setStartStateToCurrentState();
      move_group_gripper.setStartStateToCurrentState();

      // Go Home
      RCLCPP_INFO(LOGGER, "Going Home");
      joint_group_positions_arm[0] = 4.5;  // Shoulder Pan
      joint_group_positions_arm[1] = -1.57; // Shoulder Lift
      joint_group_positions_arm[2] = -1.57;  // Elbow
      joint_group_positions_arm[3] = -1.57; // Wrist 1
      joint_group_positions_arm[4] = 1.57; // Wrist 2
      joint_group_positions_arm[5] = -0.245327;  // Wrist 3
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
      bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
      move_group_arm.execute(my_plan_arm);

      RCLCPP_INFO(this->get_logger(), "Result received");
      RCLCPP_INFO(
          this->get_logger(), "X: %f, %f ",
          result.result->objects[0].object.primitive_poses[0].position.x,
          move_group_arm.getCurrentPose().pose.position.x);
      RCLCPP_INFO(
          this->get_logger(), "Y: %f, %f",
          result.result->objects[0].object.primitive_poses[0].position.y,
          move_group_arm.getCurrentPose().pose.position.y);
      RCLCPP_INFO(
          this->get_logger(), "Z: %f, %f",
          result.result->objects[0].object.primitive_poses[0].position.z,
          move_group_arm.getCurrentPose().pose.position.z);
      RCLCPP_INFO(
          this->get_logger(), "O(x): %f",
          result.result->objects[0].object.primitive_poses[0].orientation.x);
      RCLCPP_INFO(
          this->get_logger(), "O(y): %f",
          result.result->objects[0].object.primitive_poses[0].orientation.y);
      RCLCPP_INFO(
          this->get_logger(), "O(z): %f",
          result.result->objects[0].object.primitive_poses[0].orientation.z);
      RCLCPP_INFO(
          this->get_logger(), "O(w): %f",
          result.result->objects[0].object.primitive_poses[0].orientation.w);

      // Pregrasp
      RCLCPP_INFO(LOGGER, "Pregrasp Position");
      geometry_msgs::msg::Pose init_pose = move_group_arm.getCurrentPose().pose;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      //const double errx = 0.3395 - result.result->objects[0].object.primitive_poses[0].position.x;
      //const double erry = -0.0208 - result.result->objects[0].object.primitive_poses[0].position.y;

      const double errx = 0.011579;
      const double erry = -0.009977;


      RCLCPP_INFO(this->get_logger(), "ERROR - X: %f", errx);
      RCLCPP_INFO(this->get_logger(), "ERROR - Y: %f", erry);



      RCLCPP_INFO(LOGGER, "Towards X");
      std::vector<geometry_msgs::msg::Pose> approach_waypoints;
        while (
          std::abs(
              init_pose.position.x -
              result.result->objects[0].object.primitive_poses[0].position.x) >=
          0.01) {
        if (result.result->objects[0].object.primitive_poses[0].position.x >
            init_pose.position.x) {
          init_pose.position.x += 0.01;
        } else {
          init_pose.position.x -= 0.01;
        }
        approach_waypoints.push_back(init_pose);
      }
      init_pose.position.x = result.result->objects[0].object.primitive_poses[0].position.x+errx;
      approach_waypoints.push_back(init_pose);
      moveit_msgs::msg::RobotTrajectory trajectory_approach;
      double fraction = move_group_arm.computeCartesianPath(approach_waypoints, eef_step, jump_threshold, trajectory_approach);
      move_group_arm.execute(trajectory_approach);

      RCLCPP_INFO(LOGGER, "Towards Y");
      init_pose = move_group_arm.getCurrentPose().pose;
      std::vector<geometry_msgs::msg::Pose> approach_waypoints_y;
      while (
          std::abs(
              init_pose.position.y -
              result.result->objects[0].object.primitive_poses[0].position.y) >=
          0.01) {
        if (result.result->objects[0].object.primitive_poses[0].position.y >
            init_pose.position.y) {
          init_pose.position.y += 0.01;
        } else {
          init_pose.position.y -= 0.01;
        }
        approach_waypoints_y.push_back(init_pose);
      }
      init_pose.position.y = result.result->objects[0].object.primitive_poses[0].position.y+erry;
      approach_waypoints_y.push_back(init_pose);
      moveit_msgs::msg::RobotTrajectory trajectory_approach_y;
      fraction = move_group_arm.computeCartesianPath(approach_waypoints_y,eef_step, jump_threshold, trajectory_approach_y);
      move_group_arm.execute(trajectory_approach_y);

        RCLCPP_INFO(LOGGER, "Open Gripper!");
        joint_group_positions_gripper[2] = 0.0;
        move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
        bool success_gripper = (move_group_gripper.plan(my_plan_gripper) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_gripper.execute(my_plan_gripper);


        RCLCPP_INFO(LOGGER, "Approach to object!");
        init_pose = move_group_arm.getCurrentPose().pose;
        std::vector<geometry_msgs::msg::Pose> approach_waypoints_ap;
        init_pose.position.z -= 0.135;
        // init_pose.position.x += 0.011;
        // init_pose.position.y -= 0.01;
        init_pose.orientation.x = 1.0;
        init_pose.orientation.y = 0.0;
        init_pose.orientation.z = 0.0;
        init_pose.orientation.w = 0.0;
        approach_waypoints_ap.push_back(init_pose);
        moveit_msgs::msg::RobotTrajectory trajectory_approach_ap;
        fraction = move_group_arm.computeCartesianPath(approach_waypoints_ap, eef_step, jump_threshold, trajectory_approach_ap);
        move_group_arm.execute(trajectory_approach_ap);




            RCLCPP_INFO(LOGGER, "Close Gripper!");
            joint_group_positions_gripper[2] = 0.64; // 0.655
            move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
            moveit::planning_interface::MoveGroupInterface::Plan
         my_plan_gripper1; bool success_gripper1 =
         (move_group_gripper.plan(my_plan_gripper1) ==
                                     moveit::core::MoveItErrorCode::SUCCESS);
            move_group_gripper.execute(my_plan_gripper1);
            RCLCPP_INFO(LOGGER, "Retreat from object!");
            std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
            init_pose.position.z += 0.07;
            retreat_waypoints.push_back(init_pose);
            moveit_msgs::msg::RobotTrajectory trajectory_retreat;
            fraction = move_group_arm.computeCartesianPath(
                retreat_waypoints, eef_step, jump_threshold,
         trajectory_retreat); 
         move_group_arm.execute(trajectory_retreat);


        RCLCPP_INFO(LOGGER, "Rotating Arm");
      current_state_arm = move_group_arm.getCurrentState(10);
      current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                 joint_group_positions_arm);
      joint_group_positions_arm[0] -= 2.47;
      move_group_arm.setJointValueTarget(joint_group_positions_arm);
      success_arm = (move_group_arm.plan(my_plan_arm) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

      move_group_arm.execute(my_plan_arm);
      RCLCPP_INFO(LOGGER, "Release Object!");
      move_group_gripper.setNamedTarget("open");
      success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                         moveit::core::MoveItErrorCode::SUCCESS);
      move_group_gripper.execute(my_plan_gripper);
    
    
      rclcpp::shutdown();
    }
  }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetPoseClient>();
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}