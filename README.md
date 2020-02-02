# fixed_wing_formation_control

    fixed_wing_formation_control
    程序的主文件流程请看程序流程图：

    飞机监视节点在fw_control_monitor下
    飞机编队控制器在formation_controller下
    飞机的库文件在fixed_wing_lib下
    飞机的与mavros消息（topics）打包，收发有关的在pack_fw_states下
    飞机的模式切换，航点等服务（srv）不会打包，直接在task_main中调用
