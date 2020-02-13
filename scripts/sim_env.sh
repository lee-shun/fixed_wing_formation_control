###
 # @------------------------------------------1: 1------------------------------------------@
 # @Author: lee-shun
 # @Email: 2015097272@qq.com
 # @Date: 2020-02-06 17:28:05
 # @Organization: BIT-CGNC, fixed_wing_group
 # @Description:  
 # @------------------------------------------2: 2------------------------------------------@
 # @LastEditors  : lee-shun
 # @LastEditors_Email: 2015097272@qq.com
 # @LastEditTime : 2020-02-13 10:31:10
 # @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 # @LastEditors_Description:  
 # @------------------------------------------3: 3------------------------------------------@
 ###

gnome-terminal --window -e 'bash -c " roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 4; cd ~/src/Firmware; make px4_sitl gazebo_plane;  exec bash"' \
--tab -e 'bash -c "sleep 8;  rosrun fixed_wing_formation_control pack_fw_states;  exec bash"' \
