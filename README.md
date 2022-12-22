# Informative path planning for multi-robot mapping

## Description

This project focuses on the development of a method for autonomous exploration in field robotics, specifically for search, surveillance, and rescue applications. The goal is to improve the efficiency of exploration by enabling the robot to identify locations where it is likely to gain new information. To achieve this, the project first proposes a method for estimating the expected information gain at a given location in a partial map of the environment. The project then proposes a planning strategy that evaluates the informativeness of potential paths for map building purposes. To formulate a set of feasible paths two path sampling strategies are implemented and optimized using the Cross-Entropy method. The robot's sensor footprint is used to assess the information gain along a given path. The effectiveness of this algorithm is demonstrated using a single turtlebot on various partial maps.

## Usage

1. roslaunch multi_robot_sim robots_gazebo_rviz.launch
  (Launches the simulation with Gazebo for visualisation and Rviz for sensor logging
  and visualisation, and displaying maps)
2. roslaunch multi_robot_sim robots_gazebo.launch
 (Launches the simulation with Gazebo for visualisation)
3. roslaunch multi_robot_sim keyboard_teleop_robot1.launch
  (Launches teleoperative controller for robot 1)
4. roslaunch multi_robot_sim keyboard_teleop_robot2.launch
   (Launches teleoperative controller for robot 2)
5. roslaunch multi_robot_sim keyboard_teleop_robot3.launch
  (Launches teleoperative controller for robot 3)
  
6. Launch matlab code and the accompanying ros nodes on the matlab server
7. Pass robot map topic to the matlab node which would compute a path and publishes the path to turtlebots.
8. Launch m_explore multi_robot_mapping to merge the maps for each robot

## Evalauted paths for different maps for a single robot

### Path sampling via primitives

![image](https://user-images.githubusercontent.com/28558013/209092878-78e0eebe-99d0-4b1d-8b34-878fa768dab4.png)

![image](https://user-images.githubusercontent.com/28558013/209092738-908faffa-8c3a-4539-84c5-f241a2fc4d6c.png)

![image](https://user-images.githubusercontent.com/28558013/209092586-ba838a63-1151-48c1-9c59-c02a06d95105.png)


### Path sampling from a roadmap

![image](https://user-images.githubusercontent.com/28558013/209093188-743cc777-c968-415a-8fc6-8b30cbcbd9a4.png)



Code for the final year disseration at NUS
Simulation on ROS Gazebo for multi robot systems with ergodic coverage 

