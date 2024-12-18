import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess

class RvizLidarView(Node):
    def __init__(self):
        super().__init__('rviz_lidar_view')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription
        self.launch_rviz()
    
    def scan_callback(self, msg):
        self.get_logger().info('Received laser scan data')

    def launch_rviz(self):
        self.get_logger().info('Launching RViz...')
        subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2'])

    def main(args=None):
        rclpy.init(args=args)
        node = RvizLidarView()
        rclpy.spin(node)
        # node.destroy_node(node)
        rclpy.shutdown()

    if __name__ == '__main__':
        main()