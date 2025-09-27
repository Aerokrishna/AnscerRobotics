# AnscerRobotics – Multi-Map Navigation Using Wormholes

This is a repository created as a solution for the recruitment assignment provided by Anscer Robotics. 

## Overview : Problem and Solution
The problem involves a multi-map navigation system where a robot can navigate between different mapped rooms. Each room will be mapped in separate sessions.
Each room is present as a separate map file and needs to be switched during run time in order to navigate the robot.

Multi map navigation is very useful when it comes to navigating large environments where mapping and planning on a single large map is not always feasible. So we divide the map into chunks of smaller map and provide a mechanism for the robot to go from one map to another in an informed way. Examples of large environments include hospitals and offices with multiple rooms, or navigating between different floors of a building etc. In this solution I have used the wormhole mechanism. The common parts between two maps is known as a wormhole point. This is the overlapping region between two maps.The wormhole points are put in a database and retrived whenever required.

## Repository Structure

**Dependencies** : ROS2 Humble, Gazebo Fortress, sqlite3, Nav2

This repository contains four ROS2 packages.

**my_robot_interfaces** : Contains the robot interfaces necessary for ROS2 communication. In `my_robot_interfaces/action/NavGoal.action` , the structure of the action is described. We send the pose and the target map as a request, we receive feedback in the form of the current robot pose and the current map the robot is navigating in. The result includes status and a success flag to indicate whether the mission was a success or not.

**tb3+nav2** : Contains the params file, launch file and the maps necessary for nav2 navigation stack for autonomous navigation. I have used the turtlebot3 waffle robot and hence the name. The nav2_params have been tuned according to this robot. the `tb3_nav2/maps` contains all the maps which have been used. It contains 6 maps, mapA,mapB, mapC, mapD, mapE, mapF, each mapped in different sessions using the slam_toolbox. These maps represent different rooms of the turtlebot3_house world. It is a world which comes along the turtlebot3_simulations repo. The launch file contains `navigation.launch.py` whuch contains all the necessary arguments and launch files to launch the nav2 for the robot. 

**turtlebot3_gazebo** : COntains the packages to launch turtlebot3 simulation on gazebo fortress. Contains the urdf, sdf and model files necessary for the simulation as well as the robot state publisher. It also contains a `worlds` directory which contains the `turtlebot3_house.world` which was used in this demonstration. 

**wormhole_nav** : this is the package which contains the action server which contains the wormhole mechanism for multi map navigation. The logic and working of the nodes in this package have been elaborated below.

## Multi-Map Navigation : Wormhole Mechanism

The following is the flow and logic of the wormhole mechanism:

- The environment is divided into separate portions. Each portion is mapped separately using SLAM or similar methods and stored as different maps.  
- These maps are designed to have overlapping regions to facilitate transitions between portions during navigation.  
- The wormhole points and map transitions are stored in a database table with the following structure:

from_map | to_map | from_map_x | from_map_y


- `from_map` represents the current map.  
- `to_map` represents the neighboring map.  
- `from_map_x` and `from_map_y` are the wormhole points in the `from_map` frame.  

- Values are inserted into the database where `from_map` and `to_map` are the names of the maps (same as the map file names).  
- An `.db` file is obtained using SQLite3 to form this table, and it is placed in the same directory as the action server.  

---

- When the server receives an action goal (pose and target map):  
- The current map and target map are compared.  
- If they are not the same, the shortest path is computed using the Dijkstra algorithm.  
- In this algorithm, maps are considered as nodes and wormholes as edges.  
- The path is returned as a vector of strings containing map IDs, e.g., `{mapA, mapD, mapC, mapE}`.  

- The Dijkstra algorithm uses the following query to retrieve the neighbors of a given map and their corresponding wormhole coordinates (used to compute the cost of each node):

```sql
SELECT to_map_id, from_wormhole_x, from_wormhole_y
FROM wormholes WHERE from_map_id = ?;
```

- Once the map path is returned, the action server loops through it:

  - Starting with a pointer at the first index, it gets the from_map and to_map.

  - Based on this, it retrieves wormhole points as waypoints using the query:

```sql
SELECT from_wormhole_x, from_wormhole_y
FROM wormholes
WHERE from_map_id = ? AND to_map_id = ?;

```
  - On providing the from_map and to_map IDs, waypoints are obtained with respect to the from_map frame.
  
  - The from_map is then loaded, and the robot’s position is initialized.
  
  - At the beginning, it is assumed that the current map and the robot’s pose in that map are available.

- Once the robot navigates to a wormhole point:

  - The map path is checked, and the pointer is shifted to update the from_map and to_map.
  
  - New waypoints are obtained.
  
  - At the same time, the new map is loaded, and the robot’s position is initialized to the previous wormhole coordinate in the current from_map frame.

- The robot may then:

  - Navigate to the next wormhole point, or
  
  - If the current map is the target map, navigate directly to the goal.

This mechanism enables multi-map navigation.

It is optimized to minimize the Euclidean distance between wormhole points.

The robot effectively navigates through wormholes to reach the goal map and final goal coordinates.

## Action Server, Client, Dijkstra

**wormhole_server.cpp** : 
- It navigates a robot from point A to point B through multiple maps\
- This is an action server, accepts action requests in the form of target pose and target map
- If the current map is not the target map
- Creates a higher level plan by creating shortest path through maps, by referring to the databse of wormholes
- It then queries the wormhole positions to generate waypoints 
- these waypoints are sent as goals to Nav2 through the API
- Each time the map switches, the updated map is loaded and the initial pose is published for localization

**wormhole_client**: Client node sends action request in the form of goal pose and the target map name. It gets the feedback from the action server on whether the goal was acepted or not, and also the continuous feedback sent by the server. On completion of the task it receies the result as well.

**dijkstra.cpp** : 
- this script contains the functions to compute the map path
- it employs dijkstra algorithm to find the shortest path treating maps as nodes and wormholes as edges
- it queries from the database to receive the neighboring nodes and the wormhhole points.


