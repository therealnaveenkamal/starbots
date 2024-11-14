import rclpy
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
from nav2_simple_commander.robot_navigator import BasicNavigator
import threading
import time
from tf_transformations import euler_from_quaternion
import math

class NavigationNode:
    def __init__(self):
        self.node = rclpy.create_node('navigation_node')
        self.global_amcl_pose = None
        self.bt_log_status = None
        self.bt_log_lock = threading.Lock()  # Lock for accessing bt_log_status
        self.navigator = BasicNavigator()
        self.use_sim_time = True  # Assuming True by default

        self.amcl_subscriber = self.node.create_subscription(PoseWithCovarianceStamped, '/barista_1/amcl_pose', self.amcl_pose_callback, 10)
        self.bt_log_subscriber = self.node.create_subscription(BehaviorTreeLog, '/barista_1/behavior_tree_log', self.bt_log_callback, 10)
        self.client = self.node.create_client(Empty, '/barista_1/reinitialize_global_localization')
        self.ready_publisher = self.node.create_publisher(Bool, '/barista_1/ready', 10)
        ready_msg1 = Bool()
        ready_msg1.data = False
        self.ready_publisher.publish(ready_msg1)

        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/barista_1/cmd_vel', 10)

        

        # PID constants for linear and angular control
        self.kp_linear = 0.4
        self.kp_angular = 0.4
        self.target_tolerance = 0.1  # Tolerance for considering target reached


    def amcl_pose_callback(self, msg):
        self.global_amcl_pose = msg.pose.pose

    def bt_log_callback(self, msg):
        with self.bt_log_lock:
            self.bt_log_status = msg

    def reinitialize_global_localization(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service /barista_1/reinitialize_global_localization not available, waiting...')
        request = Empty.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is not None:
            self.node.get_logger().info('Global localization reinitialized successfully!')
        else:
            self.node.get_logger().error('Failed to reinitialize global localization.')

    def spin_for_duration(self, duration, angular_speed, topic):
        publisher = self.node.create_publisher(Twist, topic, 10)
        twist_msg = Twist()
        twist_msg.angular.z = angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.node.get_logger().info('Time: %f, Duration: %f' % (time.time() - start_time, duration))
            publisher.publish(twist_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        twist_msg.angular.z = 0.0
        publisher.publish(twist_msg)

    def publish_goal_pose(self, goalpose):
        publisher = self.node.create_publisher(PoseStamped, '/barista_1/goal_pose', 10)
        publisher.publish(goalpose)
        rclpy.spin_once(self.node, timeout_sec=0.5)


    def move_to_specific_point(self, target_pose):
        rate = self.node.create_rate(10)  # Publish rate
        while True:
            current_pose = self.global_amcl_pose
            print(current_pose)
            
            # Calculate linear distance to target
            linear_distance = math.sqrt((target_pose.pose.position.x - current_pose.position.x)**2 +
                                        (target_pose.pose.position.y - current_pose.position.y)**2)
            
            # Calculate angular distance to target (yaw angle)
            current_quaternion = current_pose.orientation
            target_quaternion = target_pose.pose.orientation
            current_yaw = self.quaternion_to_yaw(current_quaternion)
            target_yaw = self.quaternion_to_yaw(target_quaternion)
            angular_distance = abs(target_yaw - current_yaw)

            # Calculate control signals using PID control
            linear_velocity = self.kp_linear * linear_distance
            angular_velocity = self.kp_angular * angular_distance

            # Publish control commands
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_velocity
            cmd_vel_msg.angular.z = angular_velocity
            self.cmd_vel_publisher.publish(cmd_vel_msg)

            # Check if target reached
            if linear_distance < self.target_tolerance and angular_distance < self.target_tolerance:
                print("Achieved Target")
                break
            
            rate.sleep()

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to yaw angle (rotation about z-axis)
        # For simplicity, assuming quaternion is in (x, y, z, w) format
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        return yaw



    def print_behavior_tree_logs(self):
        while True:
            with self.bt_log_lock:
                if self.bt_log_status is not None:
                    for event in self.bt_log_status.event_log:
                        if isinstance(event, BehaviorTreeStatusChange) and event.node_name == 'NavigateRecovery' and event.previous_status == 'SUCCESS' and event.current_status == 'IDLE':
                            print("NAV2 Success")
                            ready_msg = Bool()
                            ready_msg.data = True
                            self.ready_publisher.publish(ready_msg)

            time.sleep(1)

    def start_navigation(self):
        
        spin_duration = 10
        angular_speed_left = 1.25
        angular_speed_right = -1.25
        topic = '/barista_1/cmd_vel' if self.use_sim_time else '/cmd_vel'

        if self.global_amcl_pose is not None and self.use_sim_time:
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.pose = self.global_amcl_pose
            initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = 5.0
            goal_pose.pose.position.y = -3.919
            goal_pose.pose.position.z = self.global_amcl_pose.position.z
            goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.99985, w=0.01728)

            self.publish_goal_pose(goal_pose)
            self.node.get_logger().info('Final Task Success')
        else:
            self.node.get_logger().warning('No AMCL Pose received. Cannot set the goal pose.')

    def start(self):
        cmd_vel_thread = threading.Thread(target=self.print_behavior_tree_logs)
        cmd_vel_thread.start()
        self.start_navigation()


def main():
    rclpy.init()
    node = NavigationNode()
    node.start()
    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
