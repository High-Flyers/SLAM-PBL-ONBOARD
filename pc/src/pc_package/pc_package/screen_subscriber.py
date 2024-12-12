import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ScreenSubscriber(Node):
    def __init__(self):
        super().__init__('screen_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_topic',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Camera Stream', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ScreenSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()