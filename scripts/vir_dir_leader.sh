#此脚本开启的是几何从机+仿真的从机之间的程序

gnome-terminal --window -e 'bash -c "cd ~/catkin_ws; catkin_make; exec bash"' \
--tab -e 'bash -c "sleep 2; cd ~/src/Firmware; make px4_sitl gazebo_plane;  exec bash"' \
--tab -e 'bash -c "sleep 4; roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 6; rosrun fixed_wing_formation_control pack_fw_states;  exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun fixed_wing_formation_control task_main;exec bash"' \
--tab -e 'bash -c "sleep 8; rosrun fixed_wing_formation_control vir_dir_leader;exec bash"' \






