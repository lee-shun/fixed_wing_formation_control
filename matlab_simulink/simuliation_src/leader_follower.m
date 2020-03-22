%============说明============%
%1. NED地面坐标系
%2. 只有二维平面仿真关系
%===========================%
clear;
clc;
%===
%===变量定义初始化
%===

%类宏定义
GROUP_LENGTH=200;
GRAVITY_CONSTANT=9.81;

time_stamp = 0; %时间戳，整数增加，用来数组索引
TIME = zeros(1, GROUP_LENGTH); %时间队列
now = 0.0; %当前时间
d_t = 0.1; %时间间隔
end_time = 5.0; %终止时间

%===
%===领机定义初始化
%===
%位置
led_pos_xg=zeros(1, GROUP_LENGTH);
led_pos_yg=zeros(1, GROUP_LENGTH);
led_pos_xg(1)=200;
led_pos_yg(1)=100;
%速度
led_vel_xg=zeros(1, GROUP_LENGTH);
led_vel_yg=zeros(1, GROUP_LENGTH);
led_vel_xg(1)=20.0;
led_vel_yg(1)=0.0;

%===
%===从机定义初始化
%===

%位置
fol_pos_xg=zeros(1, GROUP_LENGTH);
fol_pos_yg=zeros(1, GROUP_LENGTH);
fol_pos_xg(1)=0;
fol_pos_yg(1)=100;

%速度
fol_vel=zeros(1,GROUP_LENGTH);
fol_vel_xg=zeros(1, GROUP_LENGTH);
fol_vel_yg=zeros(1, GROUP_LENGTH);
fol_vel_xg(1)=10.0;
fol_vel_yg(1)=0;

%速度角度（偏航角）
fol_Psi=zeros(1, GROUP_LENGTH);

%误差
err_pos_xk=zeros(1, GROUP_LENGTH);
err_pos_yk=zeros(1, GROUP_LENGTH);

err_vel_xk=zeros(1, GROUP_LENGTH);
err_vel_yk=zeros(1, GROUP_LENGTH);
err_vel_k=zeros(1, GROUP_LENGTH);

err_mix_xg=zeros(1, GROUP_LENGTH);

%===
%===控制器参数
%===

k_mix_pos=0.8;%机体前向误差-位置混合系数
k_mix_vel=0.8;%机体前向误差-速度混合系数

%==
%==控制器期望值
%==

v_sp_inc=zeros(1, GROUP_LENGTH);
v_sp=zeros(1, GROUP_LENGTH);
dot_fol_Psi=zeros(1, GROUP_LENGTH);

%===
%===仿真主循环
%===

for now=0.0:d_t:end_time
    
time_stamp=time_stamp+1;
TIME(time_stamp)=now;

%===

%===
%位置误差
fol_vel_2d=[fol_vel_xg(time_stamp),fol_vel_yg(time_stamp)];
fol_vel_unit=fol_vel_2d/(norm(fol_vel_2d));
fol_vel(time_stamp)=norm(fol_vel_2d);


led_pos_2d=[led_pos_xg(time_stamp),led_pos_yg(time_stamp)];
fol_pos_2d=[fol_pos_xg(time_stamp),fol_pos_yg(time_stamp)];
err_pos_2d=led_pos_2d-fol_pos_2d;

err_pos_xk(time_stamp)=dot(fol_vel_unit,err_pos_2d);
err_pos_yk(time_stamp)=vec2cross(fol_vel_unit,err_pos_2d);

%速度误差
led_vel_2d=[led_vel_xg(time_stamp),led_vel_yg(time_stamp)];
err_vel_2d=led_vel_2d-fol_vel_2d;

TIME(time_stamp)
err_vel_2d
err_vel_xk(time_stamp)=dot(fol_vel_unit,err_vel_2d);
err_vel_yk(time_stamp)=vec2cross(fol_vel_unit,err_vel_2d);

err_vel_k(time_stamp)=sqrt(err_vel_xk(time_stamp)^2+err_vel_yk(time_stamp)^2);

%速度角度误差
if (err_vel_xk(time_stamp)==0)&&(err_vel_yk(time_stamp)>0)%正右方
    fol_Psi(time_stamp)=pi/2;
elseif (err_vel_xk(time_stamp)==0&&err_vel_yk(time_stamp)<0)%正左方
    fol_Psi(time_stamp)=-pi/2;
elseif (err_vel_xk(time_stamp)>0&&err_vel_yk(time_stamp)==0)%正前方
    fol_Psi(time_stamp)=0;
elseif(err_vel_xk(time_stamp)<0&&err_vel_yk(time_stamp)==0)%正后方
    fol_Psi(time_stamp)=pi;

elseif(err_vel_xk(time_stamp)>0&&err_vel_yk(time_stamp)>0)%右前方
    fol_Psi(time_stamp)=atan(err_vel_yk(time_stamp)/err_vel_xk(time_stamp));
elseif(err_vel_xk(time_stamp)>0&&err_vel_yk(time_stamp)>0)%左前方
    fol_Psi(time_stamp)=atan(err_vel_yk(time_stamp)/err_vel_xk(time_stamp));
elseif(err_vel_xk(time_stamp)>0&&err_vel_yk(time_stamp)>0)%右后方
    fol_Psi(time_stamp)= atan(err_vel_yk(time_stamp)/err_vel_xk(time_stamp))+pi;
elseif(err_vel_xk(time_stamp)>0&&err_vel_yk(time_stamp)>0)%左后方
    fol_Psi(time_stamp)= atan(err_vel_yk(time_stamp)/err_vel_xk(time_stamp))-pi;
end

%===
%===STEP: 获得机体前向速度期望值
%===

err_mix_xg(time_stamp)=k_mix_pos*err_pos_xk(time_stamp)+k_mix_vel*err_vel_k(time_stamp);

[err_prev,err_2prev]=find_err(time_stamp,err_mix_xg);
%按照情况选定误差，调用增量
    
v_sp_inc(time_stamp) = Incremental_PID(0.08, 0.001, 0.0, err_2prev, err_prev, err_mix_xg(time_stamp));

if(time_stamp==1)
v_sp(time_stamp)=v_sp_inc(time_stamp);
elseif(time_stamp>=2)
v_sp(time_stamp)=v_sp_inc(time_stamp)+v_sp(time_stamp-1);
end
%v_sp(time_stamp)=constrain(v_sp(time_stamp),8.0,26.0);%限幅设计
%===
%===STEP: 获得机体侧向向加速度速度期望值
%===
dot_fol_Psi(time_stamp)=0;%TODO:这里先将此处置位零，先测试速度控制器的能力

%===
%===STEP: 更新从机状态
%===

%===
%===SUB_STEP: 根据航迹侧向加速度计算地面之中的分量
%===

fol_Psi(time_stamp+1)=fol_Psi(time_stamp)+d_t*dot_fol_Psi(time_stamp);


%===
%===SUB_STEP: 更新从机速度
%===


fol_vel(time_stamp+1)=one_order_low_pass(v_sp(time_stamp),fol_vel(time_stamp),0.1);%TODO:注意，这里直接将本时刻的期望速度当做了下一时刻真是速度
fol_vel_xg(time_stamp+1)=fol_vel(time_stamp+1)*cos(fol_Psi(time_stamp+1));
fol_vel_yg(time_stamp+1)=fol_vel(time_stamp+1)*sin(fol_Psi(time_stamp+1));


%===
%===SUB_STEP: 更新从机位置
%===


fol_pos_xg(time_stamp+1)=fol_pos_xg(time_stamp)+1/2*(fol_vel_xg(time_stamp)+fol_vel_xg(time_stamp+1))*d_t;
fol_pos_yg(time_stamp+1)=fol_pos_yg(time_stamp)+1/2*(fol_vel_yg(time_stamp)+fol_vel_yg(time_stamp+1))*d_t;

%===
%===STEP: 更新领机状态
%===

%===
%===SUB_STEP: 更新领机速度
%===

led_vel_xg(time_stamp+1)=led_vel_xg(time_stamp);
led_vel_yg(time_stamp+1)=led_vel_yg(time_stamp);

led_pos_xg(time_stamp+1)=led_pos_xg(time_stamp)+1/2*(led_vel_xg(time_stamp)+led_vel_xg(time_stamp+1))*d_t;
led_pos_yg(time_stamp+1)=led_pos_yg(time_stamp)+1/2*(led_vel_yg(time_stamp)+led_vel_yg(time_stamp+1))*d_t;
%===
%===SUB_STEP: 更新领机位置
%===
end
%==
%==绘图
%==

figure(1)
grid on;

plot(TIME,fol_vel_xg,'b:',TIME,led_vel_xg,'y',TIME,err_vel_xk,'r:');
title('速度关系图');
xlabel('t s');
ylabel('V m/s');
figure(2)
grid on;

plot(TIME,fol_pos_xg,'b:',TIME,led_pos_xg,'y',TIME,err_pos_xk,'r:');
title('位置关系图');