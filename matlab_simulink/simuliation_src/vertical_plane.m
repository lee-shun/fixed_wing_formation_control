%=================================================================================
%本仿真程序的作用是竖直平面的控制器的仿真程序
%
%           （期望速度+期望高度）-->TECS控制器-->(期望油门（推力）+期望俯仰角）
%                  ^|^                                |              |
%                   |                     ---<-----<--|              |
%                   |                     |                          |
%             （质心运动学）-<-（飞行力学质心动力学第一式）<--姿态内环<--|
%
%=================================================================================

clear;
clc;

%===
%===变量定义初始化
%===

%类宏定义
GROUP_LENGTH=100;
GRAVITY_CONSTANT=9.81;

%==
%==仿真时间
%==
time_stamp = 0; %时间戳，整数增加，用来数组索引
TIME = zeros(1, GROUP_LENGTH); %时间队列
now = 0.0; %当前时间
d_t = 0.05; %时间间隔
end_time = 100; %终止时间

%==
%==领机
%==
led_height=zeros(1, GROUP_LENGTH);
led_height(1)=20;

%==
%==从机
%==
%高度
fol_height=zeros(1, GROUP_LENGTH);
fol_height(1)=10;

%速度
fol_vel=zeros(1, GROUP_LENGTH);
fol_vel(1)=10;

fol_vel_zg=zeros(1, GROUP_LENGTH);

%加速度
fol_acc=zeros(1, GROUP_LENGTH);

%俯仰角(弧度）
fol_theta=zeros(1, GROUP_LENGTH);

%==
%==期望值
%==
%速度期望值
fol_vel_sp=zeros(1, GROUP_LENGTH);

%高度期望值
fol_height_sp=zeros(1, GROUP_LENGTH);
fol_height_sp(1)=led_height(1);

%==
%==控制器
%==
param=0.1;
thrust_ingt=zeros(1, GROUP_LENGTH);%油门积分量
theta_ingt=zeros(1, GROUP_LENGTH);%俯仰角积分量
thrust_ingt(1)=0;
theta_ingt(1)=0;
thrust_sp=zeros(1, GROUP_LENGTH);%期望推力
theta_sp=zeros(1, GROUP_LENGTH);%期望俯仰角

%==
%==飞机模型参数
%==
m=2.0;%质量kg
T_max=2.0*GRAVITY_CONSTANT;%最大推力N（即4.0kg拉力）
C_d=0.02;%阻力系数
rou=1.225;%空气密度

%==
%==仿真主循环
%==

for now=0.0:d_t:end_time
    
    time_stamp=time_stamp+1;
    TIME(time_stamp)=now;
    
    %==
    %==1:计算期望值
    %==
    fol_height_sp(time_stamp)=led_height(time_stamp);
    fol_vel_sp(time_stamp)=20;

    %==
    %==2:进入TECS控制器（其中，TECS控制器的输入为：（当前高度、期望高度、当前空速、当前俯仰角、期望空速）；输出为（期望俯仰角、期望推力比例（0.0-1.0）））
    %==
    
    [theta_sp(time_stamp),thrust_sp(time_stamp),thrust_ingt,theta_ingt] =...
    TECS(fol_height,fol_height_sp,fol_vel,fol_vel_sp,fol_theta,time_stamp,d_t,thrust_ingt,theta_ingt);
    
    %==
    %==3:获得下一时刻的俯仰角，速度等信息
    %==
    
    %期望角度的处理
    fol_theta(time_stamp+1)=one_order_low_pass(theta_sp(time_stamp),fol_theta(time_stamp),0.1);
    
    %期望推力产生加速度的处理
    T=thrust_sp(time_stamp)*T_max;%推力，单位为N
    
    q=1/2*rou*fol_vel(time_stamp)^2;%动压
    
    fol_acc(time_stamp)=(1/m)*(T-q*C_d-m*GRAVITY_CONSTANT*sin(fol_theta(time_stamp)));
    
    
    %==
    %==4:更新从机速度更新位置
    %==
    
    fol_vel(time_stamp+1)=fol_vel(time_stamp)+fol_acc(time_stamp)*d_t;%更新下一个时刻的速度
    
    fol_vel_zg(time_stamp)=fol_vel(time_stamp)*sin(fol_theta(time_stamp));%计算当前时刻的z向速度
    fol_vel_zg(time_stamp+1)=fol_vel(time_stamp+1)*sin(fol_theta(time_stamp+1));%计算下一时刻的z向速度
    
    fol_height(time_stamp+1)=fol_height(time_stamp)+1/2*(fol_vel_zg(time_stamp)+fol_vel_zg(time_stamp+1))*d_t;%更新下一个时刻的高度

    %==
    %==5:更新领机位置
    %==
    led_height(time_stamp+1)=100;
end

TIME(time_stamp+1)=TIME(time_stamp)+d_t;
fol_vel_sp(time_stamp+1)=fol_vel_sp(time_stamp);
fol_height_sp(time_stamp+1)=fol_height_sp(time_stamp);

theta_sp(time_stamp+1)=theta_sp(time_stamp);
thrust_sp(time_stamp+1)=thrust_sp(time_stamp);
%==
%==绘图
%==
%速度关系图
figure(1);
plot(TIME,fol_vel,'y',TIME,fol_vel_sp,'b--','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Velocity');
xlabel('time s');
ylabel('velocity m/s');
legend('follower-velocity','setpoint-velocity');
grid on;

%高度关系图
figure(2);
plot(TIME,fol_height,'y',TIME,fol_height_sp,'b--','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Height');
xlabel('time s');
ylabel('height m');
legend('follower-height','setpoint-height');
grid on;

%角度关系图
figure(3);
plot(TIME,fol_theta,'y','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Degree');
xlabel('time s');
ylabel('angle degree');
legend('follower-theta');
grid on;

%控制量关系图
figure(4);
plot(TIME,thrust_sp,'y',TIME,theta_sp,'b--','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Control');
xlabel('time s');
ylabel('1');
legend('thrust_sp','theta_sp');
grid on;