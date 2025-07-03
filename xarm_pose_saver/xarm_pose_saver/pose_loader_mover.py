import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
import os

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.publisher = self.create_publisher(JointTrajectory, '/xarm7_traj_controller/joint_trajectory', 10)

        # Load poses
        self.pose_file = '/home/edwin/dev_ws/src/xarm_ros2/xarm_pose_saver/config/saved_joint_poses.yaml'
        if not os.path.exists(self.pose_file):
            self.get_logger().error(f"❌ Pose file not found: {self.pose_file}")
            return

        with open(self.pose_file, 'r') as f:
            self.poses = yaml.safe_load(f)

        print("✅ Available poses:", list(self.poses.keys()))
        self.run_console_loop()

    def run_console_loop(self):
        while rclpy.ok():
            pose_name = input("▶️ Enter pose name to move the arm (or 'exit' to quit): ").strip()
            if pose_name.lower() == 'exit':
                print("🔚 Exiting...")
                break

            if pose_name not in self.poses:
                print(f"⚠️ Pose '{pose_name}' not found in YAML")
                continue

            self.send_trajectory(self.poses[pose_name])

    def send_trajectory(self, joint_dict):
        msg = JointTrajectory()
        msg.joint_names = list(joint_dict.keys())

        point = JointTrajectoryPoint()
        point.positions = list(joint_dict.values())
        point.time_from_start.sec = 2  # 2 seconds to reach the target

        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"✅ Published trajectory for pose")

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
