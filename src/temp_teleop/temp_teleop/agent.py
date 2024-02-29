# Using new edge detection technique.

# Basic Modules
import os
import time
import cv2
import numpy as np
import math

# RL Modules
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import A2C
from stable_baselines3 import PPO

# Deep Learning Modules
import tensorflow as tf

# ROS Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import geometry_msgs.msg
#from gazebo_msgs.srv import ResetWorld
from nav_msgs.msg import Odometry
from raceit_interfaces.msg import InferData  # Custom message

"""
# Reset World
class WorldResetter:
    def __init__(self):
        self.node = rclpy.create_node('world_resetter_node')
        self.client = self.node.create_client(ResetWorld, '/reset_world')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting again...')

    def reset_world(self):
        request = ResetWorld.Request()

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info('Simulation world reset successfully')
        else:
            self.node.get_logger().error('Failed to reset simulation world')
"""

# To publish inference data
class InferDataPublisher(Node):

    def __init__(self):
        super().__init__('inferdata_publisher')
        self.publisher_ = self.create_publisher(InferData, 'topic', 10)

    def timer_callback(self, right_edge, left_edge):
        print("In CB:", right_edge)
        msg = InferData()

        # Converting array to 1D for transmitting
        right_edge = right_edge.flatten().tolist()
        msg.right_edge = right_edge

        left_edge = left_edge.flatten().tolist()
        msg.left_edge = left_edge

        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.right_edge)



# Subscriber to get robot position.
class OdomReceiver(Node):

    def __init__(self):
        super().__init__('odom_receiver')

        # Added
        self.odom_data = None

        self.subscription = self.create_subscription(
            Odometry,
            '/asc/odometry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)

        self.odom_data = msg.pose.pose.position



# Subscriber to get cam feed.
class CamViewSub(Node):
    def __init__(self):
        super().__init__('cam_receiver')

        # Added
        self.cam_data = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.cam_data = cv_image

# Cmd_Vel Publisher
class CmdVelPublisher(Node):
    command = [0, 0, 0, 0]
    #TwistMsg = geometry_msgs.msg.Twist
    TwistMsg = geometry_msgs.msg.TwistStamped

    def __init__(self):
        super().__init__('cmd_vel_publisher')

        self.publisher_ = self.create_publisher(self.TwistMsg, '/asc/reference', 10)
        #self.publisher_ = self.create_publisher(self.TwistMsg, '/odom', 10)
        #self.timer_callback()


    def timer_callback(self):
        print("        Callback Called")
        speed = 0.5
        turn = 1.0

        twist_msg = self.TwistMsg()
        twist = twist_msg.twist
        
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = ""

        twist_msg.header.stamp = self.get_clock().now().to_msg()
        print("~TWIST TYPE: ", type(twist))

        x = self.command[0]
        y = self.command[1]
        z = self.command[2]
        th = self.command[3]

        twist.linear.x = x * speed
        twist.linear.y = y * speed
        twist.linear.z = z * speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = th * turn
        self.publisher_.publish(twist_msg)

        # Code to prevent continuous movement
        time.sleep(0.5)

        twist.linear.x = x * 0.0
        twist.linear.y = y * 0.0
        twist.linear.z = z * 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist_msg)

class race_it_agent(gym.Env):
    args = None

    seg_model = tf.keras.models.load_model('/home/files/ros2_acker/src/temp_teleop/model/road_seg/best_weight_640_480_collab2.h5')
    
    
    def __init__(self):
        super().__init__()

        self.observation_space = spaces.Dict(
            {
                "distance_from_edge": spaces.Box(0, 1000, shape=(2,), dtype=float),
                "turn_probability": spaces.Box(0, 1000, shape=(2,), dtype=float), ## [left_road_edge_curve, right_road_edge_curve]
                "agent_position": spaces.Box(0, 1000, shape=(2,), dtype=float)
            }
        )

        self.action_space = spaces.Discrete(3)  ## Forward-0 Left-1 Right-2

        # Camera data subscriber
        rclpy.init(args=self.args)
        self.cam_view_sub = CamViewSub()

        # Robot position extractor
        self.minimal_subscriber = OdomReceiver()

        # cmd_vel Publisher
        self.cmdvel_publisher = CmdVelPublisher()

        # World reseter
        #self.resetter = WorldResetter()

        # Inference data publisher(for display)
        self.inferdata_publisher = InferDataPublisher()

        # Remove Contour Points(Contour points on the Edge"left, right, down" to be removed)
        # Image Width: 640
        # Image Height: 480
        self.rem_left = 10      # Remove those points from contour whose x-axis is less than 10.
        self.rem_right = 630    # Remove those points from contour whose x-axis is greater than 630.
        self.rem_up = 10        # Remove those points from contour whose y-axis is less than 10.
        self.rem_down = 470     # Remove those points from contour whose y-axis is greater than 470.

        # Minimum area that a contour must have to be considered for evaluation.
        self.min_cont_area = 100
        
        # Minimum length that a contour must have to be considered for evaluation.
        self.min_cont_len = 5

        # Number of pixels left and right to the road edge to check to determine weather its a left or right road edge.
        self.l_r_pix_chk = 10

        # Specifies the number of pixels to crop from the top of the image. It is included to deal espically with those
        # situations in which the road is straight and seems to merge at some place. In this case a single edge would be
        # generated combining both the left and right road edge. But if the top portion is croped, then 2 seprate road
        # edges would be detected. The value here "250" is decided after analysing such images.
        self.crop_top = 250

        # Default distance to be placed when a specific road edge is not visible in camera view.
        self.default_distance = 100  # 100 means ample space as the edge is not even visible in camera view.

        # Position of the robot in the previous timestep.
        self.prev_pose = np.zeros([2], dtype=float)    # (X, Y)
    def _get_obs(self):

        # Observation (data to be returned)
        # Format --> distance_from_edge: [[from_left_x, from_left_y], [from_right_x, from_right_y]], turn_probability: [of_left_side, of_right_side]
        observation = {"distance_from_edge": [self.default_distance, self.default_distance], "turn_probability": [0, 0], "agent_position": [0, 0]}


        # Get cam data
        rclpy.spin_once(self.cam_view_sub)
        cam_feed = self.cam_view_sub.cam_data
        
        # Temprary for testing
        #cam_feed = cv2.resize(cam_feed, (128, 128))
            
        # Preprocess image (No need to resize image as the camera output is fixed in ros2)
        cam_feed = np.expand_dims(cam_feed, axis=0)
    
        # Predict Mask
        seg_mask = self.seg_model.predict(cam_feed, verbose=1)
        seg_mask = (seg_mask > 0.5).astype(np.uint8) * 255
        
        #print("SEG Before shape: ", seg_mask.shape)
        seg_mask = np.squeeze(seg_mask)
        #print("SEG AFTER shape: ", seg_mask.shape)
        #print("Unique: ", np.unique(seg_mask))
        seg_mask = seg_mask[self.crop_top: , : ]
        
        cv2.imshow("segmented road", seg_mask)
        cv2.waitKey(1)

        mask_height, mask_width = seg_mask.shape    # batch , height, width, channel
        center_point = [int(mask_width/2), int(mask_height)]

        # Convert image to grayscale
        #gray_image = cv2.cvtColor(seg_mask, cv2.COLOR_BGR2GRAY)

        # Thresholding Image
        #_, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        binary_image = seg_mask
        
        # Inverting Image
        inverted_img = 255 - binary_image

        # Finding Contours
        contours, _ = cv2.findContours(inverted_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtering contour on the basis of area and number of points.
        contours = [contour for contour in contours if (cv2.contourArea(contour) >= self.min_cont_area) and (len(contour) > self.min_cont_len)]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
                
        
        # Squeezing extra dimensions
        for i in range(len(contours)):
            if i == 2:
                contours = contours[:2]
                break
            else:
                contours[i] = np.squeeze(contours[i])

        # Filtering edge points
        for i in range(len(contours)):
            # Extract x and y coordinates separately
            x_coords = contours[i][:, 0]
            y_coords = contours[i][:, 1]

            x_mask = np.logical_and(x_coords > self.rem_left, x_coords < self.rem_right)
            y_mask = np.logical_and(y_coords > self.rem_up, y_coords < self.rem_down)

            combined_mask = np.logical_and(x_mask, y_mask)
            contours[i] = contours[i][combined_mask]

        #print("FILTERED CONTOURS: ", contours)
        #print("FILTERED CONTOURS: ", len(contours))
        
        # Removing empty array(if they got empty after filtering edge points).
        contours = [sub_arr for sub_arr in contours if np.size(sub_arr) != 0]

        road_edge = {"right": np.array([None]), "left": np.array([None])}

        if len(contours) == 1:
            #edge_1 = np.squeeze(contours[0])
            #edge_1 = np.unique(edge_1, axis=0)
            #edge_1 = edge_1[edge_1[:, 1].argsort()]

            edge_1 = contours[0]
            edge_1 = edge_1[edge_1[:, 1].argsort()]

            # Deciding right and left road edge.
            mid_edge_1 = edge_1[int(len(edge_1) / 2)]

            if (binary_image[mid_edge_1[1]][mid_edge_1[0] - self.l_r_pix_chk] == 0) and (binary_image[mid_edge_1[1]][mid_edge_1[0] + self.l_r_pix_chk] == 255):
                road_edge['left'] = edge_1
            elif (binary_image[mid_edge_1[1]][mid_edge_1[0] - self.l_r_pix_chk] == 255) and (binary_image[mid_edge_1[1]][mid_edge_1[0] + self.l_r_pix_chk] == 0):
                road_edge['right'] = edge_1
            else:
                print("ERROR WHEN CHECKING LANE. EXITING......")
                #exit()

        elif (len(contours) >= 2):
            edge_1 = contours[0]
            edge_1 = edge_1[edge_1[:, 1].argsort()]

            #edge_2 = np.squeeze(contours[1])
            #edge_2 = np.unique(edge_2, axis=0)
            #edge_2 = edge_2[edge_2[:, 1].argsort()]

            edge_2 = contours[1]
            edge_2 = edge_2[edge_2[:, 1].argsort()]

            # For Debugging
            #print("edge1: ", edge_1)
            #print("len edge1: ", len(edge_1))

            #print("edge2: ", edge_2)
            #print("len edge2: ", len(edge_2))

            #print("CONTOURS: ", contours)

            # Deciding right and left road edge.
            mid_edge_1 = edge_1[int(len(edge_1) / 2)]
            mid_edge_2 = edge_2[int(len(edge_2) / 2)]

            if (binary_image[mid_edge_1[1]][mid_edge_1[0] - self.l_r_pix_chk] == binary_image[mid_edge_2[1]][mid_edge_2[0] + self.l_r_pix_chk] == 255) and (binary_image[mid_edge_1[1]][mid_edge_1[0] + self.l_r_pix_chk] == binary_image[mid_edge_2[1]][mid_edge_2[0] - self.l_r_pix_chk] == 0):
                road_edge['left'] = edge_2
                road_edge['right'] = edge_1
            elif (binary_image[mid_edge_1[1]][mid_edge_1[0] + self.l_r_pix_chk] == binary_image[mid_edge_2[1]][mid_edge_2[0] - self.l_r_pix_chk] == 255) and (binary_image[mid_edge_1[1]][mid_edge_1[0] - self.l_r_pix_chk] == binary_image[mid_edge_2[1]][mid_edge_2[0] + self.l_r_pix_chk] == 0):
                road_edge['left'] = edge_1
                road_edge['right'] = edge_2
            else:
                print("ERROR WHEN CHECKING LANE. EXITING......")
                #exit()

        #print("Edge Type Left: ", type(road_edge['left']))
        #print("Left Val:", road_edge['left'])
        #print("Edge Type Right: ", type(road_edge['right']))
        #print("Right Val:", road_edge['right'])

        if road_edge['left'].all() != None:
            # Calculating Distance from the center
            dist_left = self.euclidean_distance(road_edge['left'][-1], center_point)
            # Checking if the lowest point of the left road segment is left to the center of the robot.
            if road_edge['left'][-1][0] > center_point[0]:
                dist_left = dist_left * -1

            observation['distance_from_edge'][0] = dist_left

            # Calculating Turn Probability
            x_data = road_edge['left'][:, 0]
            y_data = road_edge['left'][:, 1]
            left_curvature = self.calculate_curvature(x_data, y_data)

            observation['turn_probability'][0] = np.sum(left_curvature)/len(left_curvature)


        if road_edge['right'].all() != None:
            # Calculating Distance from the center
            dist_right = self.euclidean_distance(road_edge['right'][-1], center_point)
            # Checking if the lowest point of the right road segment is right to the center of the robot.
            if road_edge['right'][-1][0] < center_point[0]:
                dist_right = dist_right * -1

            observation['distance_from_edge'][1] = dist_right

            # Calculating Turn Probability
            x_data = road_edge['right'][:, 0]
            y_data = road_edge['right'][:, 1]
            right_curvature = self.calculate_curvature(x_data, y_data)

            observation['turn_probability'][1] = np.sum(right_curvature)/len(right_curvature)

            #print("TYPE RC: ", type(right_curvature))

        # Extracting robot position
        # Get data(robot position) from "/odom".
        rclpy.spin_once(self.minimal_subscriber)

        print("@@@@@@@@@@@@@@@~")

        bot_pose = np.zeros([2], dtype=float)  # (X, Y)
        #bot_pose[0] = self.minimal_subscriber.odom_data.x
        #bot_pose[1] = self.minimal_subscriber.odom_data.y
        
        
        
        # round(number, 4)
        bot_pose[0] = round(self.minimal_subscriber.odom_data.x, 2)
        bot_pose[1] = round(self.minimal_subscriber.odom_data.y, 2)
        
     
        
        observation["agent_position"] = bot_pose

        # Sending data to display.
        self.inferdata_publisher.timer_callback(road_edge['left'], road_edge['right'])


        print("RETURNING OBSERVATION: ", observation)
        
        print("DONE")
        
        return observation

    def _get_info(self):
        # Return info ro debugging purpose.
        temp_info = {"Nice": 1}
        return temp_info

    def reset(self, seed=None):
        #self.resetter.reset_world()

        obseravtion = self._get_obs()
        info = self._get_info()

        return obseravtion, info

    def step(self, action):
        print("Took Action:", action)
        # Convert RL action to ros2 specific action and Publish the new position to "/cmd_vel".
        action = self.process_action(action)

        # Get new observation from "/odom"
        observation = self._get_obs()
        print("Got Observation")

        info = self._get_info()

        # Calculate Reward
        reward, terminated = self.total_reward(observation, action)
        print("Getting Reward")

        return observation, reward, terminated, False, info

    def euclidean_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance

    def calculate_curvature(self, x, y):
        dx_dt = np.gradient(x)
        dy_dt = np.gradient(y)
        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)

        curvature = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2) / ((dx_dt ** 2 + dy_dt ** 2) ** 1.5)

        return curvature

    def process_action(self, action):
        ## Forward-0 Left-1 Right-2.
        avl_actions = [(1, 0, 0, 0), (0, 0, 0, 1), (0, 0, 0, -1)]  # Up, Left, Right.

        x = avl_actions[action][0]
        y = avl_actions[action][1]
        z = avl_actions[action][2]
        th = avl_actions[action][3]
        self.cmdvel_publisher.command = [x, y, z, th]
        print("    Processing Action:", self.cmdvel_publisher.command)
        # rclpy.spin_once(self.cmdvel_publisher)
        # rclpy.spin(self.cmdvel_publisher)
        self.cmdvel_publisher.timer_callback()

    def total_reward(self, observation, action):
        # Reward:
        #   +2*Distance --> For each unit of Distance Traveled
        #   -1 --> Not Moving  (Have selected it as constant for now because the speed of the robot is also constant.)
        #   -10 --> Moving out of track. (When the distance to any road edge becomes negative)
        #   -0.20 -> for every steer.

        total_reward = 0
        terminated = False

        # Reward on the basis of distance traveled.
        print("Reward for distance: ")
        
        print("ERROR: obs, prev_obs", observation["agent_position"], self.prev_pose)
        print("ERROR: obs, prev_obs", len(observation["agent_position"]), len(self.prev_pose))

        if (observation["agent_position"][0] == self.prev_pose[0]) and (observation["agent_position"][1] == self.prev_pose[1]):
            total_reward = total_reward - 1
            print("    Negative: -1")
        else:
            total_reward = total_reward + (2 * self.euclidean_distance(observation["agent_position"], self.prev_pose))
            self.prev_pose = observation["agent_position"]
            print("    Positive: ", total_reward)

        # Reward for preventing excessive steering.

        ## Forward-0 Left-1 Right-2.
        if action == 1 or action == 2:
            total_reward = total_reward - 0.20
            print("Reward for excessive steering: -0.20")

        # Reward for moving out off the track.
        if (observation['distance_from_edge'][0] < 0) or (observation['distance_from_edge'][1] < 0):
            total_reward = total_reward - 10
            terminated = True
            print("Reward for Moving out of the track: -10")

        print("TOTAL REWARD: ", total_reward)
        print("TERMINATED: ", terminated)
        return total_reward, terminated

def main():
    env = race_it_agent()

    # PPO
    model = PPO("MultiInputPolicy", env, verbose=1)
    model.learn(total_timesteps=10_000)
    print("Learning Completed!")
    
    
    
"""
# Temp Code

# Train the model from begning
def main():
    env = race_it_agent()

    # PPO
    model = PPO("MultiInputPolicy", env, verbose=1)
    
    # Train the model
    total_timesteps = 10000
    save_interval = 1000  # Save the model every 1000 iterations
    
    for i in range(0, total_timesteps, save_interval): 
        model.learn(total_timesteps=save_intervals, reset_num_timesteps=False)
        
        # Save the model after every save_interval iterations
        if i > 0:
            model.save(f"ppo_model_iter_{i}")
        
    print("Learning Completed!")
    
# Continue training the model from whem where it left off last time and 
def main():    
    model_path = ""
    model = model = PPO.load(model_path)
    
    # Train the model
    total_timesteps = 10000
    save_interval = 1000  # Save the model every 1000 iterations
    
    for i in range(0, total_timesteps, save_interval): 
        model.learn(total_timesteps=save_intervals, reset_num_timesteps=False)
        
        # Save the model after every save_interval iterations
        if i > 0:
            model.save(f"ppo_model_iter_{i}")
        
    print("Learning Completed!")

"""

