import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
import cv2
import numpy as np

from raceit_interfaces.msg import InferData

class InferDataSubscriber(Node):

    def __init__(self):
        super().__init__('inferdata_subscriber')
        self.infer_data = {'right_edge': None, 'left_edge':None}
        self.subscription = self.create_subscription(
            InferData,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        right_edge = np.array(msg.right_edge, dtype=np.uint32)
        left_edge = np.array(msg.left_edge, dtype=np.uint32)

        right_shape = (right_edge.size //2, 2)
        left_shape = (left_edge.size //2, 2)

        #print("@@@@@@@@@@@@@@@@@@@")
        #print(right_edge.shape)
        #print(left_edge.shape)
        #print("left shape:", left_shape)
        #print("right shape:", right_shape)
        #print("@@@@@@@@@@@@@@@@@@@")


        self.infer_data['right_edge'] = right_edge.reshape(right_shape)
        self.infer_data['left_edge'] = left_edge.reshape(left_shape)


        #self.infer_data = msg
        #self.get_logger().info('I heard: "%s"' % msg.right_edge[2])


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')

        # Cam data
        self.data = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        # self.image_count = 0
        # self.save_dir = "/home/pratham/Desktop/temp/race_it_images"

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.data = cv_image
        # save_path = os.path.join(self.save_dir, str(self.image_count)+".png")
        # print("Path exists: ", os.path.exists(save_path))
        # cv2.imwrite(save_path, cv_image)
        # self.image_count += 1

        # self.get_logger().info('Image saved at: "%s"' % save_path)


def main(args=None):

    crop_top = 250
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    inferdata_subscriber = InferDataSubscriber()
    #rclpy.spin(minimal_subscriber)

    while True:
        rclpy.spin_once(minimal_subscriber) # Fetching camera data

        rclpy.spin_once(inferdata_subscriber) # Fetching inference data

        infer_data = inferdata_subscriber.infer_data
        disp_img = minimal_subscriber.data
            
        

        left_edge = infer_data['left_edge']
        left_edge = left_edge.astype(np.int32)

        right_edge = infer_data['right_edge']
        right_edge = right_edge.astype(np.int32)

        #print("List:")
        #print(type(t1[0][0]))
        #print(t2)

        #cv2.polylines(disp_img, [infer_data['left_edge']], True, (0, 255, 0), 2)
        #cv2.polylines(disp_img, [infer_data['right_edge']], True, (0, 0, 255), 2)
        
        cv2.polylines(disp_img[crop_top:, :], [left_edge], isClosed=True, color=(0, 255, 0), thickness=2)
        cv2.polylines(disp_img[crop_top:, :], [right_edge], isClosed=True, color=(0, 0, 255), thickness=2)

        print("Img: ", type(disp_img))
        print("Shape: ", disp_img.shape)

        cv2.imshow("Realtime_View", disp_img)

        # Check for key press (wait for 1 millisecond)
        key = cv2.waitKey(1)

        # If 'q' is pressed, exit the loop
        if key == ord('q'):
            break

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
exit
