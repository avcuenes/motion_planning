
## Clean up the build project
.PHONY : clean 
clean:
	rm -rf build/ install/ log/ 

## Build all project
.PHONY : build_all
build_all:
	colcon build

## Build just home point publisher
.PHONY : build_home_point
build_home_point:
	colcon build --packages-select home_point_pub

## Build just target point publisher
.PHONY : build_target_point
build_target_point:
	colcon build --packages-select target_point_pub

## Build just rrt start algorithm 
.PHONY : build_rrt_start
build_rrt_start:
	colcon build --packages-select rrt_star

## Build just obstacle point publisher 
.PHONY : build_obstacle_point_publisher
build_obstacle_point_publisher:
	colcon build --packages-select obstacle_location

## Source project
.PHONY : source_project
source_project:
	bash install/setup.bash

## Run just home point publisher
.PHONY : run_home_point
run_home_point:
	@make source_project
	ros2 run home_point_pub homepoint

## Run just home point publisher
.PHONY : run_target_point
run_target_point:
	@make source_project
	ros2 run target_point_pub targetpoint
