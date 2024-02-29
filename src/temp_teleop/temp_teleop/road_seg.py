import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

import time

class ImageSubscriber:
    def __init__(self):
        self.node = rclpy.create_node('image_subscriber')
        self.subscription = self.node.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # QoS profile, adjust as needed
        )
        self.bridge = CvBridge()
        self.image_count = 0
        self.save_directory = "/home/pratham/Desktop/temp/race_it_images"
        
        self.start_time = time.time()
        self.wait_time = 5
        print("Save path exists: ", os.path.exists(self.save_directory))

    def image_callback(self, msg):
        
        if (time.time() - self.start_time) > self.wait_time:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save the image
            image_filename = f'image_{self.image_count}.png'
            image_path = os.path.join(self.save_directory, image_filename)
            cv2.imwrite(image_path, cv_image)
            self.image_count += 1
            self.node.get_logger().info(f'Saved image: {image_path}')
            
            self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber.node)
    image_subscriber.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
