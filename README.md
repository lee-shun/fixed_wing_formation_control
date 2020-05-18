## ✈✈ _固定翼编队控制器设计及实现_

### 整体逻辑
![avatar](./graduation_thesis/figures/c4/c4-soft-hard.png)
### 文件结构
    .
    ├── CMakeLists.txt          ===》     编译文件
    ├── compile_commands.json   ===》     clang文件
    ├── config                  ===》     ROS节点配置文件
    ├── flight_logs             ===》     典型的飞行仿真数据
    ├── graduation_thesis       ===》     论文目录
    ├── launch                  ===》     启动文件
    ├── matlab_simulink         ===》     matlab仿真文件
    ├── msg                     ===》     自定义消息文件
    ├── package.xml             ===》     包描述文件
    ├── processing_frame        ===》     程序流程图
    ├── README.md               ===》     自述
    ├── scripts                 ===》     一些脚本文件
    └── src                     ===》     编队控制器源程序

### 编队控制器软件逻辑
![avatar](./graduation_thesis/figures/c4/c4-soft-hard.png)

### 依赖项目
##### 1. 运行环境
[ROS](https://www.ros.org/) --开源机器人操作系统
##### 2. 依赖
[mavros]() 功能包 </br>
[serial]() 功能包 </br>

##### 3. 姿态内环
[PX4](https://px4.io/) 自动驾驶仪 </br>

### 注意
1. 仅作为兴趣以及学术交流使用，*请勿做商用*

2. 本软件可以被更改以及使用，*请注明出处*

### 最后
本项目的进步与发展，得益于北京理工大学飞行器动力学与控制实验室（BIT-
CGNC）的对我的培养与帮助。</br>另外，由于我受到帮助以及指点实在
太多，详情请参见论文致谢页。
若有志同道合的朋友，欢迎交流！
