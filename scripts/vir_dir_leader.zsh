###
 # @------------------------------------------1: 1------------------------------------------@
 # @Author: lee-shun
 # @Email: 2015097272@qq.com
 # @Date: 2020-02-06 17:28:05
 # @Organization: BIT-CGNC, fixed_wing_group
 # @Description:  #此脚本开启的是几何从机+仿真的从机之间的程序 
 # @------------------------------------------2: 2------------------------------------------@
 # @LastEditors: lee-shun
 # @LastEditors_Email: 2015097272@qq.com
 # @LastEditTime: 2020-02-19 18:07:13
 # @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 # @LastEditors_Description:  zsh
 # @------------------------------------------3: 3------------------------------------------@
 ###

gnome-terminal --window -e 'zsh -c "cd ~/catkin_ws; catkin_make; exec zsh"' \
--tab -e 'zsh -c "sleep 2; cd ~/src/Firmware; make px4_sitl gazebo_plane;  exec zsh"' \
--tab -e 'zsh -c "sleep 4; roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec zsh"' \
--tab -e 'zsh -c "sleep 6; rosrun fixed_wing_formation_control pack_fw_states;  exec zsh"' \
--tab -e 'zsh -c "sleep 10; rosrun fixed_wing_formation_control task_main;exec zsh"' \
--tab -e 'zsh -c "sleep 8; rosrun fixed_wing_formation_control vir_dir_leader;exec zsh"' \






