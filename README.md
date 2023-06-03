# Object Detection on ArduPilot SITL 

## Requirements :
* Ubuntu (20.04 LTS) Full 3D graphics hight recommended.
* Gazebo version 11
* ROS Noetic
* MAVROS

## MAVROS installation :

MAVLink extendable communication node for ROS with proxy for Ground Control Station (See original instructions here http://ardupilot.org/dev/docs/ros-install.html#installing-mavros).

### Configure your Ubuntu repositories
````
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

cd ~/

wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

chmod a+x install_geographiclib_datasets.sh

./install_geographiclib_datasets.sh
````

## STEP 1 - SITL Ardupilot installation :

Instructions taken from ardupilot.org (See original instructions here http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).

### Clone ArduPilot repository

````
cd ~/
git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git submodule update --init --recursive
````

### Install some required packages

If you are on a debian based system (such as Ubuntu or Mint), we provide a script that will do it for you. From ardupilot directory :

````
Tools/scripts/install-prereqs-ubuntu.sh -y
````

Reload the path (log-out and log-in to make permanent):

````
. ~/.profile
````


### Finalize and test the installation

To start the simulator first change directory to the vehicle directory. For example, for the multicopter code change to ardupilot/ArduCopter:


````
cd ~/ardupilot/ArduCopter
````

Then start the simulator using sim_vehicle.py. The first time you run it you should use the -w option to wipe the virtual EEPROM and load the right default parameters for your vehicle.

````
sim_vehicle.py --console --map
````

### Updating MAVProxy and pymavlink

New versions of MAVProxy and pymavlink are released quite regularly. If you are a regular SITL user you should update every now and again using this command

````
sudo pip install --upgrade pymavlink MAVProxy
````


This concludes the first step SITL Ardupilot installation. 

## STEP 2 - Ardupilot gazebo plugin installation :

(See original instructions here https://github.com/khancyr/ardupilot_gazebo).

### Clone ArduPilot repository

````
cd ~/

git clone https://github.com/r0ch1n/ardupilot_gazebo

cd ardupilot_gazebo

mkdir build

cd build

cmake ..

# use make without any parameter if running in a VM
make -j4

sudo make install
````

### Set environment variables

````
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

````

Set Path of Gazebo Models (Adapt the path to where to clone the repo)
````
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
````

Set Path of Gazebo Worlds (Adapt the path to where to clone the repo)
````
echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
````

Reload the path (log-out and log-in to make permanent):

````
source ~/.bashrc
````

### Test installation

Open one Terminal and launch SITL Ardupilot
````
sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console
````

Open a second Terminal and launch Gazebo running ardupilot_gazebo plugin
````
gazebo --verbose worlds/iris_arducopter_runway.world
````

You should see a gazebo world with a small quadcopter right at the center


### Setup MAVROS

New versions of MAVProxy and pymavlink are released quite regularly. If you are a regular SITL user you should update every now and again using this command

````
cd ~/

mkdir -p ardupilot_ws/src

cd ardupilot_ws

catkin init

cd src

mkdir launch

cd launch

roscp mavros apm.launch apm.launch

sudo gedit apm.launch

To connect to SITL we just need to modify the first line to <arg name="fcu_url" default="udp://127.0.0.1:14551@14555" />. save you file and launch it with
````

### Test

````
cd ~/ardupilot_ws/src/launch

roslaunch apm.launch
````

## Run it all

First source your catkin workspace wherein you have cloned the repository

### 1. Run ArduPilot SITL 

````
gazebo --verbose worlds/iris_arducopter_runway.world
````

### 2. Initialize Gazebo Simulation

````
roslaunch ardupilot_gazebo iris_with_roscam.launch
````
### 3. Run square_movement node

````
rosrun communication setDestination
````

### 4. Run darknet_ros

````
roslaunch darknet_ros darknet_ros.launch 
````

<hr>

## Example
<img src="simulation.jpeg">

## Troubleshooting
### 1. udp0: sendto: Invalid argument mavros
Please check this <a href="https://discuss.ardupilot.org/t/problem-calling-ros-service-for-arducopter-ros-sitl-tutorial/18992">blog</a> 

### 2. link 1 down, even after launching ardupilot_gazebo
Check X-Terminal's Output (if two X-Terminals open). The only remedy I could find was to reinstall ardupilot SITL and ardupilot_gazebo

### 3. Could not find a package configuration file provided by "mavros" with any   of the following names:      mavrosConfig.cmake     mavros-config.cmake
Follow the installation mentioned in this <a href="https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation">link</a>
