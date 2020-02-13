###
 # @------------------------------------------1: 1------------------------------------------@
 # @Author: lee-shun
 # @Email: 2015097272@qq.com
 # @Date: 2020-02-06 17:28:05
 # @Organization: BIT-CGNC, fixed_wing_group
 # @Description:  #此脚本的作用是进行自定义消息的打印 
 # @------------------------------------------2: 2------------------------------------------@
 # @LastEditors  : lee-shun
 # @LastEditors_Email: 2015097272@qq.com
 # @LastEditTime : 2020-02-13 10:31:01
 # @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 # @LastEditors_Description:  
 # @------------------------------------------3: 3------------------------------------------@
 ###

gnome-terminal --window -e 'bash -c "rosrun rqt_graph rqt_graph; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/fw_states; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/leader_states; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/formation_control_states; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/fw_cmd; exec bash"' \
