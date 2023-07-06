# sudo apt-get update
# sudo apt-get upgrade
# sudo reboot
# sudo dphys-swapfile swapoff
# sudo nano /etc/dphys-swapfile (edit file CONF_SWAPSIZE=2048)
# sudo dphys-swapfile setup
# sudo dphys-swapfile swapon
# free -h
# $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# cat /etc/apt/sources.;ist.d/ros-latest.list
# sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# sudo apt update
# $ sudo apt-get install python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall build-essential cmake
# sudo rosdep init
# rosdep update
# mkdir ~/ros_catkin_ws
# cd ~/ros_catkin_ws
# rosinstall_generator desktop --rosdistro noetic --deps --wet-only --tar > noetic-desktop.rosinstall
# wstool init src noetic-desktop-wet.rosinstall
# sudo pip3 install -U rosdep rospkg catkin_pkg
# sudo apt-get install python3-empy
# sudo apt-get install libogre-1.9-dev libogre-1.9.0v5
# rosdep install --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
# sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
# echo "source /opt/ros/noetic/setup.bash" >> ~/ .bashrc
# source ~/ .bashrc
# roscore 
# rosrun turtlesim turtlesim_node (in new terminal)
# rosrun turtlesim turtle_teleop_key (in new terminal)