import subprocess
import time
import signal
import rclpy
from std_msgs.msg import Bool

# Global variable to hold references to subprocesses
sub_processes = []

def wait_for_topic(node, topic, timeout=180):
    start_time = time.time()
    while not node.has_parameter(topic):
        if time.time() - start_time > timeout:
            return False
        time.sleep(1)
    return True

def kill_subprocesses():
    for proc in sub_processes:
        proc.kill()

def signal_handler(sig, frame):
    print("\nTerminating subprocesses...")
    kill_subprocesses()
    exit(0)


def main():
    subprocess.Popen(["ros2", "launch", "arm_real", "move_group.launch.py"])
    time.sleep(5)
    subprocess.Popen(["ros2", "launch", "moveit2_scripts", "pick_hole0.launch.py", "use_sim_time:=False"])
    time.sleep(70)


if __name__ == '__main__':
    main()
