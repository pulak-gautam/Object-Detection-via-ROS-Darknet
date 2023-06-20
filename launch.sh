#!/bin/bash

gnome-terminal --tab -- bash -c "gazebo --verbose /home/atulya/Workspaces/robofest23/src/Object-Detection-via-ROS-Darknet/communication/worlds/iris_arducopter_runway.world"

echo "Press ENTER once Gazebo launched"
read $1

gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I1 --out=tcpin:0.0.0.0:8100 " 
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I2 --out=tcpin:0.0.0.0:8200 " 
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I3 --out=tcpin:0.0.0.0:8300 "
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I4 --out=tcpin:0.0.0.0:8400 " 
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone5 -I5 --out=tcpin:0.0.0.0:8500 " 


echo "Press ENTER to connect the simulation"
read $1 


gnome-terminal --tab -- bash -c "roslaunch communication apm.launch"


echo "Press ENTER to launch the simulation"
read $1 

gnome-terminal --tab -- bash -c "roslaunch communication fly.launch"
