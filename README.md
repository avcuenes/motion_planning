# motion_planning

This project have written for the motion planning for car-like vehicles.

## Project Description

At this project, we use the following algorithm for motion planning. Kinodynamics RRT* motion planning algorithm implemented.
And also, we design the software to implement the other motion planning algorithms. 


## Folder Structure

```markdown
motion_planning/
│
├── Makefile # Make commands 
└── src/
│   ├── home_point_pub     # Home point publisher project
│   └── map_constraint_pub # Map constraint publisher project
│   └── obstacle_location  # Obstacle point  publisher project
│   └── rrt_star           # Motion Planner project   
│   └── target_point_pub   # Target point publisher project
|   └── visulization       # Visulization project

```
## Projects 
There is a few projects to simulate real life. Projects ros-graph is like the following pictures;

![](rosgraph.svg)

### home_point_pub
This project just publishes home point location information.

### map_constraint_pub
This project just publishes map constraint information.

### obstacle_location
This project just publishes obstacle location information.

### rrt_star
This project calculate path with respect to the given information from home point, target point, map constraint and obstacle location topics.

### target_point_pub
This project publishes target point location information

### visualization
This project visualize the start and end point, paths generated from rrt_star project and obstacle location.

## Kinodynamic RRT* 



## How to Run the Project

We test this project using the following ros2 foxy distro and ubuntu 20.04.