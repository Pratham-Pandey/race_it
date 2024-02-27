# RaceIt  🏎️


## About

* The goal of "Race_It" is to make a car cover a track as fast as possible.
* "RaceIt" is inspired from AWS DeepRacer.

## Inside The Black Box

RaceIt operates on the following techology:

* ### ROS2
    * Facilitates communication between various components of RaceIt, enabling seamless data exchange and coordination among sensors, actuators, and decision-making modules.
   
* ### Gazebo
    * RaceIt utilizes the Gazebo simulator to create realistic virtual environments for testing and training.

* ### Reinforcement Learning
    * Reinforcement Learning (RL) serves as the brain of RaceIt, enabling the autonomous racing car to learn and adapt to its environment through trial and error. RaceIt employs Proximal Policy Optimization (PPO), a state-of-the-art RL algorithm, to train the racing car's decision-making policies based on rewards and penalties received during simulation and real-world operation.

* ### Computer Vision
    * Computer vision plays a critical role in RaceIt by providing sensory information about the racing environment. RaceIt utilizes U-Net, a convolutional neural network architecture, for road segmentation, enabling the racing car to identify and navigate through the racing track with precision and accuracy.

## Run Locally

Clone the project
```bash
  git clone https://link-to-project
```

Go to the project directory
```bash
  cd my-project
```

Build project
```bash
  colcon build --symlink-install        
```

Source project
```bash
  source install/setup.bash
```

Load simulation environment
```bash
  ros2 launch race_it rsp.launch.py  
```

Load On Screen Display
```bash
  ros2 run temp_teleop display_road
```

Initialise Agent
```bash
  ros2 run temp_teleop agent
```

## Screenshots

![App Screenshot](https://via.placeholder.com/468x300?text=App+Screenshot+Here)
![Label](NEW IMG)




## Some Links That Might Help

If you have recently installed ros2 and are new to robotics, the project might not run at first go due to missing dependencies. The following are the links to the questions that might help:

* https://robotics.stackexchange.com/questions/108074/no-module-named-roslaunch-for-ros-2
* https://robotics.stackexchange.com/questions/108174/error-loader-for-controller-asc-not-found
