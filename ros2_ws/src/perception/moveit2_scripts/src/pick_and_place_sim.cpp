#include <chrono>
#include <cstddef>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdlib>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

bool isObjectReady = false;

void readyCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    RCLCPP_INFO(LOGGER, "Object is ready, releasing the gripper.");
    // Release the object here
    isObjectReady = true; // Set the flag
  }
}

double x_target = 0;
double y_target = 0;

bool cupDetected = false;

void conePositionCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  x_target = msg->x;
  y_target = msg->y;
  cupDetected = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto cone_position_sub =
      move_group_node->create_subscription<geometry_msgs::msg::Point>(
          "cone_position", 10, conePositionCallback);

  while (!cupDetected) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(LOGGER, "Pose: %f, %f", x_target, y_target);

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);
  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);
  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);
  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();



// Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "ground";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2.0;
    primitive.dimensions[primitive.BOX_Y] = 1.0;
    primitive.dimensions[primitive.BOX_Z] = 0.1;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.z = 0.707;
    box_pose.orientation.w = 0.707;
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.6;
    box_pose.position.z = -0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object1 = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object1;
    collision_object1.header.frame_id = frame_id;
    collision_object1.id = "coffee_machine";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.8;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.z = 0.707;
    box_pose.orientation.w = 0.707;
    box_pose.position.x = 0.3;
    box_pose.position.y = 0.9;
    box_pose.position.z = 0;

    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(box_pose);
    collision_object1.operation = collision_object1.ADD;

    return collision_object1;
  }();

    auto collision_object2 = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = frame_id;
    collision_object2.id = "cup";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.05;
    primitive.dimensions[primitive.BOX_Y] = 0.05;
    primitive.dimensions[primitive.BOX_Z] = 0.15;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.25;
    box_pose.position.y = 0.41;
    box_pose.position.z = 0.06;

    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(box_pose);
    collision_object2.operation = collision_object2.ADD;

    return collision_object2;
  }();


auto collision_object3 = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object3;
    collision_object3.header.frame_id = frame_id;
    collision_object3.id = "barista";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.2;
    primitive.dimensions[primitive.BOX_Y] = 0.2;
    primitive.dimensions[primitive.BOX_Z] = 0.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.3;
    box_pose.position.y = 0.1;
    box_pose.position.z = -0.6;

    collision_object3.primitives.push_back(primitive);
    collision_object3.primitive_poses.push_back(box_pose);
    collision_object3.operation = collision_object3.ADD;

    return collision_object3;
  }();



  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(collision_object1);
  planning_scene_interface.applyCollisionObject(collision_object2);
  planning_scene_interface.applyCollisionObject(collision_object3);


  std::this_thread::sleep_for(std::chrono::seconds(5));



  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");
  joint_group_positions_arm[0] = 4.5;       // Shoulder Pan
  joint_group_positions_arm[1] = -1.57;     // Shoulder Lift
  joint_group_positions_arm[2] = -1.57;     // Elbow
  joint_group_positions_arm[3] = -1.57;     // Wrist 1
  joint_group_positions_arm[4] = 1.57;      // Wrist 2
  joint_group_positions_arm[5] = -0.245327; // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double z_target = 0.320;
  RCLCPP_INFO(LOGGER, "Towards z");
  std::vector<geometry_msgs::msg::Pose> approach_waypoints_z;
  geometry_msgs::msg::Pose init_pose = move_group_arm.getCurrentPose().pose;
  while (std::abs(init_pose.position.z - z_target) >= 0.01) {
    if (z_target > init_pose.position.z) {
      init_pose.position.z += 0.01;
    } else {
      init_pose.position.z -= 0.01;
    }
    approach_waypoints_z.push_back(init_pose);
    RCLCPP_INFO(LOGGER, "Z Pose: %f", init_pose.position.z);
  }
  init_pose.position.z = z_target;
  approach_waypoints_z.push_back(init_pose);
  moveit_msgs::msg::RobotTrajectory trajectory_approach_z;
  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints_z, eef_step, jump_threshold, trajectory_approach_z);
  move_group_arm.execute(trajectory_approach_z);

  // double y_target1 = 0.411;
  RCLCPP_INFO(LOGGER, "Towards Y");
  init_pose = move_group_arm.getCurrentPose().pose;
  std::vector<geometry_msgs::msg::Pose> approach_waypoints_y;
  while (std::abs(init_pose.position.y - y_target) >= 0.01) {
    if (y_target > init_pose.position.y) {
      init_pose.position.y += 0.01;
    } else {
      init_pose.position.y -= 0.01;
    }
    approach_waypoints_y.push_back(init_pose);
  }
  init_pose.position.y = y_target;
  approach_waypoints_y.push_back(init_pose);
  moveit_msgs::msg::RobotTrajectory trajectory_approach_y;
  fraction = move_group_arm.computeCartesianPath(
      approach_waypoints_y, eef_step, jump_threshold, trajectory_approach_y);
  move_group_arm.execute(trajectory_approach_y);

  RCLCPP_INFO(LOGGER, "Open Gripper!");
  joint_group_positions_gripper[0] = 0.399;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);

  // double x_target1 = 0.246;
  RCLCPP_INFO(LOGGER, "Towards X");
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  init_pose = move_group_arm.getCurrentPose().pose;
  while (std::abs(init_pose.position.x - x_target) >= 0.01) {
    if (x_target > init_pose.position.x) {
      init_pose.position.x += 0.01;
    } else {
      init_pose.position.x -= 0.01;
    }
    approach_waypoints.push_back(init_pose);
    RCLCPP_INFO(LOGGER, "X Pose: %f", init_pose.position.x);
  }
  init_pose.position.x = x_target;
  approach_waypoints.push_back(init_pose);
  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);
  move_group_arm.execute(trajectory_approach);

  RCLCPP_INFO(LOGGER, "Approach to object!");
  geometry_msgs::msg::Pose target_pose1 = move_group_arm.getCurrentPose().pose;
  std::vector<geometry_msgs::msg::Pose> approach_waypoints_approach;
  target_pose1.position.z -= 0.007;
  target_pose1.position.x += 0.001;
  approach_waypoints_approach.push_back(target_pose1);
  moveit_msgs::msg::RobotTrajectory trajectory_approach_approach;
  fraction = move_group_arm.computeCartesianPath(approach_waypoints_approach,
                                                 eef_step, jump_threshold,
                                                 trajectory_approach_approach);
  move_group_arm.execute(trajectory_approach_approach);

  RCLCPP_INFO(LOGGER, "Close Gripper");
  joint_group_positions_gripper[0] = 0.10163182289561132; // 0.1043;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper3;
  bool success_gripper1 = (move_group_gripper.plan(my_plan_gripper3) ==
                           moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper3);
  

  moveit_msgs::msg::AttachedCollisionObject attached_object;
    collision_object2.operation = collision_object2.ADD;
  attached_object.link_name = "wrist_3_link";
  attached_object.object = collision_object2;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  std::this_thread::sleep_for(std::chrono::seconds(5));


  RCLCPP_INFO(LOGGER, "Retreat from object!");
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.x -= 0.069;
  target_pose1.position.y -= 0.002;
  target_pose1.position.z += 0.044;
  retreat_waypoints.push_back(target_pose1);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
  move_group_arm.execute(trajectory_retreat);

  RCLCPP_INFO(LOGGER, "Rotating Arm");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  joint_group_positions_arm[0] = 0.184;
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);


  RCLCPP_INFO(LOGGER, "Going to Barista");
  joint_group_positions_arm[0] = 0.18492282308549157; // Shoulder Pan
  joint_group_positions_arm[1] = -1.8068571191118394; // Shoulder Lift
  joint_group_positions_arm[2] = -1.5292020705886724; // Elbow
  joint_group_positions_arm[3] = -1.3988706546611152; // Wrist 1
  joint_group_positions_arm[4] = 1.571433325229509;   // Wrist 2
  joint_group_positions_arm[5] = 0.06236559051888957; // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  RCLCPP_INFO(LOGGER, "Aligning Gripper");

  // Execute motion to retreat from hole3
  //-0.418, 0.196, -0.203
  //-0.284, 0.234, -0.207

  std::vector<geometry_msgs::msg::Pose> hole3wp_z;
  geometry_msgs::msg::Pose target_pose_hole3_z =
      move_group_arm.getCurrentPose().pose;
  target_pose_hole3_z.position.z -= 0.466;
  target_pose_hole3_z.position.y += 0.054;
  target_pose_hole3_z.position.x += 0.040;

  hole3wp_z.push_back(target_pose_hole3_z);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat_hole3_z;
  fraction = move_group_arm.computeCartesianPath(
      hole3wp_z, eef_step, jump_threshold, trajectory_retreat_hole3_z);
  move_group_arm.execute(trajectory_retreat_hole3_z);


  RCLCPP_INFO(LOGGER, "Release Object!");
  move_group_gripper.setNamedTarget("open");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);
  
  collision_object2.operation = collision_object2.REMOVE;
  attached_object.link_name = "wrist_3_link";
  attached_object.object = collision_object2;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  std::this_thread::sleep_for(std::chrono::seconds(5));


  std::vector<geometry_msgs::msg::Pose> hole3wp_up;
  geometry_msgs::msg::Pose target_pose_hole3 =
      move_group_arm.getCurrentPose().pose;
  target_pose_hole3.position.z += 0.6;
  hole3wp_up.push_back(target_pose_hole3);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat_hole3_up;
  fraction = move_group_arm.computeCartesianPath(
      hole3wp_up, eef_step, jump_threshold, trajectory_retreat_hole3_up);
  move_group_arm.execute(trajectory_retreat_hole3_up);

  RCLCPP_INFO(LOGGER, "Executing master.launch.py...");
  // Execute the launch file
  //std::system("python3 /home/user/ros2_ws/src/master.launch.py");

  // Shutdown the node
  RCLCPP_INFO(LOGGER, "Shutting down...");
  
  rclcpp::shutdown();
  return 0;
}