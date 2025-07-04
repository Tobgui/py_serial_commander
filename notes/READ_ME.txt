

sudo apt update
sudo apt install \
  ros-humble-robot-localization \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control 
sudo apt install slam_toolbox


colcon build titi --merge-isntall
source install/setup.bash

ros2 launch articubot_one launch_sim.launch.py
ros2 launch py_serial_commander serial_commander.launch.py 

//if the robot didnt move or problem at launch
sudo lsof /dev/ttyACM0
sudo kill -9 <output from previous command
pkill -f serial_commander
pkill -f python

//if gazebo didnt work properly----------------------------------------------------------------
pkill -9 gzserver
pkill -9 gzclient
pkill -9 spawn_entity.py 
sudo rm -rf /dev/shm/*