#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__("image_subscriber")
        self.subscription = self.create_subscription(Image, '/image_tuning', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # Display the image
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(1)  # Wait for a short duration, needed for the image to be displayed properly

        except Exception as e:
            self.get_logger().error("Error processing the image: {0}".format(str(e)))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
