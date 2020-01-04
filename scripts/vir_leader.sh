#!/bin/sh

gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin build; exec bash"' \
--tab -e 'bash -c "sleep 8; cd ~/src/Firmware; make px4_sitl gazebo_plane;  exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 15; rosrun mavros vir_leader; exec bash"' \
--tab -e 'bash -c "sleep 20; roslaunch mavlink_uart mavlink_uart_send.launch; exec bash"' \



#--tab -e 'bash -c "sleep 10; roscore; exec bash"' \
# -e 'bash -c "sleep 2; make posix gazebo_plane; exec bash"' \






