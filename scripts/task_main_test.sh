gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin_make; sleep 18; rosrun rqt_graph rqt_graph; exec bash"' \
--tab -e 'bash -c "sleep 8; cd ~/src/Firmware; make px4_sitl gazebo_plane;  exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 12; rosrun fixed_wing_formation_control pack_fw_states; exec bash"' \
--tab -e 'bash -c "sleep 13; rosrun fixed_wing_formation_control task_main; exec bash"' 
#--tab -e 'bash -c "cd ..; ./QGroundControl.AppImage;exec bash"'
