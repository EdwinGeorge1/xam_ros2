import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml
import os
import threading

class MultiPoseConsoleSaver(Node):
    def __init__(self):
        super().__init__('multi_pose_console_saver')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.latest_joint_data = None
        self.pose_request_queue = []
        self.save_file = '/home/edwin/dev_ws/src/xarm_ros2/xarm_pose_saver/config/saved_joint_poses.yaml'

        # 🧵 Start input thread
        input_thread = threading.Thread(target=self.console_input_loop)
        input_thread.daemon = True
        input_thread.start()

    def console_input_loop(self):
        while True:
            pose_name = input("📝 Enter a name to save current pose (or 'exit' to quit): ").strip()
            if pose_name.lower() == 'exit':
                print("🔚 Exiting...")
                os._exit(0)
            elif pose_name:
                self.pose_request_queue.append(pose_name)

    def joint_callback(self, msg: JointState):
        self.latest_joint_data = dict(zip(msg.name, msg.position))

        if self.pose_request_queue:
            pose_name = self.pose_request_queue.pop(0)
            self.save_pose(pose_name)

    def save_pose(self, pose_name):
        if self.latest_joint_data is None:
            self.get_logger().warn("⚠️ No joint data received yet.")
            return

        # Load existing data if any
        if os.path.exists(self.save_file):
            with open(self.save_file, 'r') as f:
                try:
                    poses = yaml.safe_load(f) or {}
                except yaml.YAMLError:
                    poses = {}
        else:
            poses = {}

        # Save new pose
        poses[pose_name] = self.latest_joint_data
        with open(self.save_file, 'w') as f:
            yaml.dump(poses, f)

        self.get_logger().info(f"✅ Pose '{pose_name}' saved.")

def main():
    rclpy.init()
    node = MultiPoseConsoleSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
