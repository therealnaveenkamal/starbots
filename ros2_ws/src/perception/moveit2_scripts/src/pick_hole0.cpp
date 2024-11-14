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

  /*
    while (!cupDetected) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    */

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
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.6;
    box_pose.position.z = -0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const collision_object1 = [frame_id =
                                      move_group_arm.getPlanningFrame()] {
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
    primitive.dimensions[primitive.BOX_Z] = 0.10;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.08;
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
    primitive.dimensions[primitive.BOX_X] = 0.4;
    primitive.dimensions[primitive.BOX_Y] = 0.4;
    primitive.dimensions[primitive.BOX_Z] = 0.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.4;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.65;

    collision_object3.primitives.push_back(primitive);
    collision_object3.primitive_poses.push_back(box_pose);
    collision_object3.operation = collision_object3.ADD;

    return collision_object3;
  }();

  auto collision_object4 = [frame_id = move_group_arm.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object4;
    collision_object4.header.frame_id = frame_id;
    collision_object4.id = "wall";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 2.0;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.z = 0.707;
    box_pose.orientation.w = 0.707;
    box_pose.position.x = -0.2;
    box_pose.position.y = -0.4;
    box_pose.position.z = 0.2;

    collision_object4.primitives.push_back(primitive);
    collision_object4.primitive_poses.push_back(box_pose);
    collision_object4.operation = collision_object4.ADD;

    return collision_object4;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(collision_object1);
  //planning_scene_interface.applyCollisionObject(collision_object2);
  planning_scene_interface.applyCollisionObject(collision_object3);
  planning_scene_interface.applyCollisionObject(collision_object4);

  std::this_thread::sleep_for(std::chrono::seconds(5));

RCLCPP_INFO(LOGGER, "Open Gripper");
  joint_group_positions_gripper[0] = 0.2;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper_finalopen;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper_finalopen) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper_finalopen);


  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");
  joint_group_positions_arm[0] = 3.1292314529418945;   // Shoulder Pan
  joint_group_positions_arm[1] = 0.6202115255543212;  // Shoulder Lift
  joint_group_positions_arm[2] = 0.5338438192950647;   // Elbow
  joint_group_positions_arm[3] = -2.7405735454955042;    // Wrist 1
  joint_group_positions_arm[4] = -1.5312121550189417;  // Wrist 2
  joint_group_positions_arm[5] = -0.11431485811342412; // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);


  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  



  RCLCPP_INFO(LOGGER, "Close Gripper");
  joint_group_positions_gripper[0] = -0.01; // 0.1043;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper3;
  bool success_gripper1 = (move_group_gripper.plan(my_plan_gripper3) ==
                           moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper3);

  RCLCPP_INFO(LOGGER, "Retreat from object!");
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  geometry_msgs::msg::Pose target_pose1 = move_group_arm.getCurrentPose().pose;
  target_pose1.position.z += 0.6;
  retreat_waypoints.push_back(target_pose1);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  double fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
  move_group_arm.execute(trajectory_retreat);


  RCLCPP_INFO(LOGGER, "Going to Barista");
  joint_group_positions_arm[0] = 2.9293444; // Shoulder Pan
  joint_group_positions_arm[1] = -0.818586; // Shoulder Lift
  joint_group_positions_arm[2] = 0.1562646; // Elbow
  joint_group_positions_arm[3] = -0.916542; // Wrist 1
  joint_group_positions_arm[4] = -1.528683; // Wrist 2
  joint_group_positions_arm[5] = -0.314582; // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);


  RCLCPP_INFO(LOGGER, "Going to Drop X");
  joint_group_positions_arm[0] = 1.0642445087432861;   // Shoulder Pan
  joint_group_positions_arm[1] = -1.087642417555191;  // Shoulder Lift
  joint_group_positions_arm[2] = 0.7685125509845179;   // Elbow
  joint_group_positions_arm[3] = -1.2666642528823395;    // Wrist 1
  joint_group_positions_arm[4] = -1.6014555136310022;  // Wrist 2
  joint_group_positions_arm[5] = -2.164046589528219; // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  std::vector<geometry_msgs::msg::Pose> hole3wp_y;
  geometry_msgs::msg::Pose target_pose_hole3_y =
      move_group_arm.getCurrentPose().pose;
  target_pose_hole3_y.position.z -= 0.055;
  hole3wp_y.push_back(target_pose_hole3_y);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat_hole3_y;
  fraction = move_group_arm.computeCartesianPath(
      hole3wp_y, eef_step, jump_threshold, trajectory_retreat_hole3_y);
  move_group_arm.execute(trajectory_retreat_hole3_y);

  RCLCPP_INFO(LOGGER, "Release Object!");
  joint_group_positions_gripper[0] = 0.2;
  move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
  success_gripper = (move_group_gripper.plan(my_plan_gripper_finalopen) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper_finalopen);
  

  std::vector<geometry_msgs::msg::Pose> hole3wp_up;
  geometry_msgs::msg::Pose target_pose_hole3 =
      move_group_arm.getCurrentPose().pose;
  target_pose_hole3.position.z += 0.06;
  hole3wp_up.push_back(target_pose_hole3);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat_hole3_up;
  fraction = move_group_arm.computeCartesianPath(
      hole3wp_up, eef_step, jump_threshold, trajectory_retreat_hole3_up);
  move_group_arm.execute(trajectory_retreat_hole3_up);
  
  RCLCPP_INFO(LOGGER, "Going to Barista");
  joint_group_positions_arm[0] = 2.9293444; // Shoulder Pan
  joint_group_positions_arm[1] = -0.818586; // Shoulder Lift
  joint_group_positions_arm[2] = 0.1562646; // Elbow
  joint_group_positions_arm[3] = -0.916542; // Wrist 1
  joint_group_positions_arm[4] = -1.528683; // Wrist 2
  joint_group_positions_arm[5] = -0.314582; // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);



  /*
    std::vector<geometry_msgs::msg::Pose> hole3wp_x;
    geometry_msgs::msg::Pose target_pose_hole3_x =
        move_group_arm.getCurrentPose().pose;
    target_pose_hole3_x.position.x += 0.048;
    target_pose_hole3_x.position.x += 0.032;
    hole3wp_x.push_back(target_pose_hole3_x);
    moveit_msgs::msg::RobotTrajectory trajectory_retreat_hole3_x;
    fraction = move_group_arm.computeCartesianPath(
        hole3wp_x, eef_step, jump_threshold, trajectory_retreat_hole3_x);
    move_group_arm.execute(trajectory_retreat_hole3_x);


    /*
      auto clock = move_group_node->get_clock();

      std::shared_ptr<tf2_ros::Buffer> tf_buffer;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

      tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

      geometry_msgs::msg::TransformStamped transformStamped;
      double X_DIFF, Y_DIFF;

      while (rclcpp::ok()) {

        try {
          transformStamped =
              tf_buffer->lookupTransform("tool0", "hole1", rclcpp::Time(0));
          X_DIFF = transformStamped.transform.translation.x;
          Y_DIFF = transformStamped.transform.translation.y;
          if (!std::isnan(X_DIFF) && !std::isnan(Y_DIFF)) {
            break; // Break out of loop if valid values are obtained
          }
        } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(LOGGER, "Transform lookup failed: %s", ex.what());
        }
      }

      // Compute the difference in x and y

      std::vector<geometry_msgs::msg::Pose> hole3wp_th;
      geometry_msgs::msg::Pose target_pose_hole3_th =
          move_group_arm.getCurrentPose().pose;
      target_pose_hole3_th.position.x += 0.108;
      target_pose_hole3_th.position.y += 0.004;
      hole3wp_th.push_back(target_pose_hole3_th);
      moveit_msgs::msg::RobotTrajectory trajectory_retreat_hole3_th;
      fraction = move_group_arm.computeCartesianPath(
          hole3wp_th, eef_step, jump_threshold, trajectory_retreat_hole3_th);
      move_group_arm.execute(trajectory_retreat_hole3_th);

      std::this_thread::sleep_for(std::chrono::seconds(10));

      RCLCPP_INFO(LOGGER, "Pose: %f, %f", X_DIFF, Y_DIFF);
      RCLCPP_INFO(LOGGER, "Pose: %f, %f", X_DIFF, Y_DIFF);
      RCLCPP_INFO(LOGGER, "Pose: %f, %f", X_DIFF, Y_DIFF);
      RCLCPP_INFO(LOGGER, "Pose: %f, %f", X_DIFF, Y_DIFF);
      RCLCPP_INFO(LOGGER, "Pose: %f, %f", X_DIFF, Y_DIFF);
      RCLCPP_INFO(LOGGER, "Pose: %f, %f", X_DIFF, Y_DIFF);





        RCLCPP_INFO(LOGGER, "Executing master.launch.py...");
        // Execute the launch file
        std::system("python3 /home/user/ros2_ws/src/master.launch.py");

        // Shutdown the node
        */
  RCLCPP_INFO(LOGGER, "Shutting down...");
  rclcpp::shutdown();
  return 0;
}