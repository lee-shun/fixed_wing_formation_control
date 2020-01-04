#!/bin/sh

gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin build; sleep 18; rosrun rqt_graph rqt_graph; exec bash"' \
--tab -e 'bash -c "sleep 8; cd ~/src/1.8.2/Firmware; make posix gazebo_plane;  exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 12; rosrun mavros fixed_wing_formation_control; exec bash"' \
--tab -e 'bash -c "cd ..; ./QGroundControl.AppImage;exec bash"'



#--tab -e 'bash -c "sleep 10; roscore; exec bash"' \
# -e 'bash -c "sleep 2; make posix gazebo_plane; exec bash"' \






