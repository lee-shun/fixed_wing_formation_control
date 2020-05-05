#本脚本启动的是双机（三机）的编队仿真

#zsh
gnome-terminal --window  -e 'zsh -c "cd ~/catkin_ws; catkin_make; exec zsh"' \
--tab -e 'zsh -c "sleep 2; source ~/.zshrc; roslaunch px4 multi_uav_mavros_sitl_sdf.launch;  exec zsh"' \
--tab -e 'zsh -c "sleep 4; rosrun fixed_wing_formation_control pack_leader_states; exec zsh"' \
--tab -e 'zsh -c "sleep 6; rosrun fixed_wing_formation_control vir_sim_leader;  exec zsh"' \
--tab -e 'zsh -c "sleep 10; rosrun fixed_wing_formation_control pack_fw1_states;exec zsh"' \
--tab -e 'zsh -c "sleep 8; rosrun fixed_wing_formation_control follower1_main;exec zsh"' \
--tab -e 'zsh -c "sleep 8; rosrun fixed_wing_formation_control switch_fw1_mode;exec zsh"' \
--tab -e 'zsh -c "sleep 12; cd ~/rosbag; rosbag record -O lee /uav1/fixed_wing_formation_control/formation_control_states" ' \

