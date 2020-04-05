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
d_t = 0.05; %时间间隔
end_time = 200; %终止时间

%==
%==领机
%==
led_height=zeros(1, GROUP_LENGTH);
led_height(1)=100;

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

%输出量
throttle_setpoint=zeros(1, GROUP_LENGTH);%期望推力
pitch_setpoint=zeros(1, GROUP_LENGTH);%期望俯仰角

%控制器内部状态量（需要记录）
STE_error=zeros(1, GROUP_LENGTH);
STE_rate_error=zeros(1, GROUP_LENGTH);
throttle_integ_state=zeros(1, GROUP_LENGTH);

SEB_error= zeros(1, GROUP_LENGTH);
SEB_rate_error= zeros(1, GROUP_LENGTH);
pitch_integ_state=zeros(1, GROUP_LENGTH);

%==
%==飞机模型参数
%==
m=2.0;%质量kg
T_max=2.0*GRAVITY_CONSTANT;%最大推力N（即4.0kg拉力）
C_d=0.046;%阻力系数
rou=1.225;%空气密度

%==
%==仿真主循环
%==

for now=0.0:d_t:end_time
    
    time_stamp=time_stamp+1;
    TIME(time_stamp)=now;
    
    %==
    %==1:得到期望值
    %==
    fol_height_sp(time_stamp)=led_height(time_stamp);
    fol_vel_sp(time_stamp)=20;

    %==
    %==2:进入TECS控制器
    %==
    
    %0.得到前面的值
    if (time_stamp==1)
    prev=1;%前一个标志位
    else
        prev=time_stamp-1;
    end

    %1.计算变化率
    hgt_rate_setpoint=1/(d_t)*(fol_height_sp(time_stamp)-fol_height_sp(prev));
    vel_rate_setpoint=1/(d_t)*(fol_vel_sp(time_stamp)-fol_vel_sp(prev));
    
    fol_vel_rate=1/(d_t)*(fol_vel(time_stamp)-fol_vel(prev));
    fol_vert_vel=fol_vel(time_stamp)*sin(fol_theta(time_stamp));
    
    %2.计算动能以及势能
    SPE_setpoint=fol_height_sp(time_stamp)*GRAVITY_CONSTANT;
    SKE_setpoint=0.5*(fol_vel_sp(time_stamp))^2;
    
    SPE_rate_setpoint=hgt_rate_setpoint*GRAVITY_CONSTANT;
    SKE_rate_setpoint = fol_vel(time_stamp) * vel_rate_setpoint;%注意此处的定义
    
    SPE_estimate=fol_height(time_stamp)*GRAVITY_CONSTANT;
    SKE_estimate=0.5*(fol_vel(time_stamp))^2;
    
    SPE_rate=fol_vert_vel*GRAVITY_CONSTANT;
    SKE_rate=fol_vel(time_stamp)*fol_vel_rate;
    
    %3.计算油门值
    STE_error(time_stamp)=(SPE_setpoint+SKE_setpoint)-(SPE_estimate+SKE_estimate);
    
    %油门值更新参数计算
    max_climb_rate=10.0;
    min_sink_rate=3.0;
    throttle_setpoint_max=0.74;
    throttle_setpoint_min=0.05;
    throttle_cruise=0.25;
    throttle_time_constant=8.0;
    throttle_damping_gain=0.01;
    integrator_gain_height=0.08;
    
    STE_rate_max=max_climb_rate*GRAVITY_CONSTANT;
    STE_rate_min=min_sink_rate*GRAVITY_CONSTANT;
    
    STE_rate_setpoint=constrain((SKE_rate_setpoint+SPE_rate_setpoint),STE_rate_min,STE_rate_max);
    STE_rate_error(time_stamp) = 0.2 * (STE_rate_setpoint - SPE_rate - SKE_rate) + 0.8 * STE_rate_error(prev); %one_order_filiter

    if (STE_rate_setpoint >= 0) %计算前馈
    throttle_predicted = throttle_cruise + STE_rate_setpoint / STE_rate_max * (throttle_setpoint_max - throttle_cruise);
    else
    throttle_predicted = throttle_cruise + STE_rate_setpoint / STE_rate_min * (throttle_setpoint_min - throttle_cruise);
    end
    STE_to_throttle = 1.0 / (throttle_time_constant * (STE_rate_max - STE_rate_min));
    
    throttle_setpoint(time_stamp) = (STE_error(time_stamp) + STE_rate_error(time_stamp) * throttle_damping_gain) *...
    STE_to_throttle + throttle_predicted;
    
    %计算积分项
    throttle_integ_state(time_stamp) = throttle_integ_state(prev)+(STE_error(time_stamp)*integrator_gain_height)*d_t*STE_to_throttle;

    throttle_setpoint(time_stamp) = throttle_setpoint(time_stamp) + throttle_integ_state(time_stamp);

    throttle_setpoint(time_stamp)=constrain(throttle_setpoint(time_stamp),throttle_setpoint_min,throttle_setpoint_max);
    
    
    %4.计算俯仰角
    pitch_time_constant=10.0;
    pitch_damping_gain=0.01;
    pitch_setpoint_min=-pi/6;
    pitch_setpoint_max=pi/4;
    vert_accel_limit=8.0;
    integrator_gain_pitch=0.08;
    
    SEB_setpoint=SPE_setpoint-SKE_setpoint;
    SEB_rate_setpoint=SPE_rate_setpoint-SKE_rate_setpoint;
    
    SEB_error(time_stamp) = SEB_setpoint - (SPE_estimate  - SKE_estimate);
	SEB_rate_error(time_stamp) = SEB_rate_setpoint - (SPE_rate - SKE_rate);

    climb_angle_to_SEB_rate = fol_vel(time_stamp) * pitch_time_constant * GRAVITY_CONSTANT;
    pitch_integ_input = SEB_error(time_stamp) * integrator_gain_pitch;
    pitch_integ_state(time_stamp) = pitch_integ_state(prev) + pitch_integ_input * d_t;

    SEB_correction = SEB_error(time_stamp) + SEB_rate_error(time_stamp) * pitch_damping_gain + SEB_rate_setpoint * pitch_time_constant;

    pitch_setpoint_unc = (SEB_correction + pitch_integ_state(time_stamp)) / climb_angle_to_SEB_rate;
	pitch_setpoint(time_stamp) = constrain(pitch_setpoint_unc, pitch_setpoint_min, pitch_setpoint_max);

    
%     ptchRateIncr = d_t * vert_accel_limit / fol_vel(time_stamp);
% 
% 	if ((pitch_setpoint(time_stamp) - pitch_setpoint(prev)) > ptchRateIncr)
% 	
% 		pitch_setpoint(time_stamp) = pitch_setpoint(prev) + ptchRateIncr;
% 	
%     elseif ((pitch_setpoint(time_stamp) - pitch_setpoint(prev)) < ptchRateIncr)
% 	
% 		pitch_setpoint(time_stamp) = pitch_setpoint(prev) - ptchRateIncr;
% 	
%     end

    
    %==
    %==3:获得下一时刻的俯仰角，速度等信息
    %==
    
    %期望角度的处理
    fol_theta(time_stamp+1)=one_order_low_pass(pitch_setpoint(time_stamp),fol_theta(time_stamp),0.1);
    
    %期望推力产生加速度的处理
    T=throttle_setpoint(time_stamp)*T_max;%推力，单位为N
    
    q=0.5*rou*fol_vel(time_stamp)^2;%动压
    
    fol_acc(time_stamp)=(1/m)*(T-q*C_d-m*GRAVITY_CONSTANT*sin(fol_theta(time_stamp)));
    
    
    %==
    %==4:更新从机速度、位置
    %==
    
    fol_vel(time_stamp+1)=fol_vel(time_stamp)+fol_acc(time_stamp)*d_t;%更新下一个时刻的速度
    
    fol_vel_zg(time_stamp)=fol_vel(time_stamp)*sin(fol_theta(time_stamp));%计算当前时刻的z向速度
    fol_vel_zg(time_stamp+1)=fol_vel(time_stamp+1)*sin(fol_theta(time_stamp+1));%计算下一时刻的z向速度
    
    fol_height(time_stamp+1)=fol_height(time_stamp)+1/2*(fol_vel_zg(time_stamp)+fol_vel_zg(time_stamp+1))*d_t;%更新下一个时刻的高度

    %==
    %==5:更新领机位置
    %==
    led_height(time_stamp+1)=led_height(time_stamp);
end

TIME(time_stamp+1)=TIME(time_stamp)+d_t;
fol_vel_sp(time_stamp+1)=fol_vel_sp(time_stamp);
fol_height_sp(time_stamp+1)=fol_height_sp(time_stamp);

pitch_setpoint(time_stamp+1)=pitch_setpoint(time_stamp);
throttle_setpoint(time_stamp+1)=throttle_setpoint(time_stamp);
%==
%==绘图
%==
%速度关系图
TIME=TIME*0.5;
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
plot(TIME,throttle_setpoint,'y',TIME,pitch_setpoint,'b--','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Control');
xlabel('time s');
ylabel('1');
legend('throttle_setpoint','theta_sp');
grid on;