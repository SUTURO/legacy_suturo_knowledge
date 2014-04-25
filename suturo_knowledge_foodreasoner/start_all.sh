#!/bin/bash

roslaunch suturo_perception_rosnode run_bag.launch
rosbag play ~/suturo_ws/src/suturo_perception/bags/kinect-raw-cup_tape_box.bag --loop
roslaunch suturo_manipulation_gazebo gazebo_pr2_test_simulation.launch world:=pr2_table_three_beer.world
