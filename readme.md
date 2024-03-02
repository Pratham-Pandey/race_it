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
  git clone https://github.com/Pratham-Pandey/race_it.git
```

Go to the project directory
```bash
  cd race_it
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

## Output Video


https://github.com/Pratham-Pandey/race_it/assets/85839299/5b30796e-40c0-453c-b547-297f234cf075



## Screenshots
### *Overview Screen*
![overview_screen](https://github.com/Pratham-Pandey/race_it/assets/85839299/0a9b42e6-2c48-4f60-8367-daf7021cfe88)

### *Gazebo Screen*
![main_screen](https://github.com/Pratham-Pandey/race_it/assets/85839299/95ebe1ac-7dbc-47a2-a9b7-a4824226fd5e)

### *Rviz Screen*
![rviz](https://github.com/Pratham-Pandey/race_it/assets/85839299/b14d23d8-fd54-4c10-9b6d-9d538cd45075)

### *Camera View*
![cam_view](https://github.com/Pratham-Pandey/race_it/assets/85839299/520f6815-2a84-42a0-b1c3-070bc99cbc56)

### *Road Segmented View*
![segmented_road](https://github.com/Pratham-Pandey/race_it/assets/85839299/3a78cb7f-ff09-4e77-8dc4-10501fa12593)








## Some Links That Might Help

If you have recently installed ros2 and are new to robotics, the project might not run at first go due to missing dependencies. The following are the links to the questions that might help:

* https://robotics.stackexchange.com/questions/108074/no-module-named-roslaunch-for-ros-2
* https://robotics.stackexchange.com/questions/108174/error-loader-for-controller-asc-not-found
