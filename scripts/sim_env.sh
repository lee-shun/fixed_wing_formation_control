###
 # @Author: lee-shun
 # @Email: 2015097272@qq.com
 # @Date: 2020-02-06 17:28:05
 # @Organization: BIT-CGNC, fixed_wing_group
 # @Description:  
 ###
gnome-terminal --window -e 'bash -c " roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14557"; exec bash"' \
--tab -e 'bash -c "sleep 4; cd ~/src/Firmware; make px4_sitl gazebo_plane;  exec bash"' \
--tab -e 'bash -c "sleep 8;  rosrun fixed_wing_formation_control pack_fw_states;  exec bash"' \
