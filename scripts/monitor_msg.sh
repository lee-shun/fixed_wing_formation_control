#此脚本的作用是进行自定义消息的打印
gnome-terminal --window -e 'bash -c "rosrun rqt_graph rqt_graph; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/fw_states; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/leader_states; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/formation_control_states; exec bash"' \
--tab -e 'bash -c "rostopic echo /fixed_wing_formation_control/fw_cmd; exec bash"' \
