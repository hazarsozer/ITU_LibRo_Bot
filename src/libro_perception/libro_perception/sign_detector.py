import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        
        # Initialize the bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to the robot's camera topic
        # NOTE: We might need to change '/camera/image_raw' later depending on the robot!
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Sign Detector Node has started!')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # For now, just prove we have the image
            # We will add detection logic here later!
            self.get_logger().info(f'Received image frame: {cv_image.shape}')
            
            # Optional: Show the image in a window (Popups can be tricky in some setups)
            # cv2.imshow("Robot Camera", cv_image)
            # cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
