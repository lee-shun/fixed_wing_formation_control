%============说明============%
%1. NED地面坐标系
%2. 只有二维平面仿真关系
%3. 本脚本仿真角度控制，速度方面尽量不用到
%===========================%
clear;
clc;
%===
%===变量定义初始化
%===

%类宏定义
GROUP_LENGTH=50;
GRAVITY_CONSTANT=9.81;

time_stamp = 0; %时间戳，整数增加，用来数组索引
TIME = zeros(1, GROUP_LENGTH); %时间队列
now = 0.0; %当前时间
d_t = 0.05; %时间间隔
end_time = 30; %终止时间

%===
%===领机定义初始化
%===
%领机位置
led_pos_xg=zeros(1, GROUP_LENGTH);
led_pos_yg=zeros(1, GROUP_LENGTH);
led_pos_xg(1)=40;
led_pos_yg(1)=100;
%领机速度
led_vel_xg=zeros(1, GROUP_LENGTH);
led_vel_yg=zeros(1, GROUP_LENGTH);
led_vel_xk=zeros(1, GROUP_LENGTH);
led_vel_yk=zeros(1, GROUP_LENGTH);
led_vel_xg(1)=15.0;
led_vel_yg(1)=0.0;

%===
%===从机定义初始化
%===

%从机位置
fol_pos_xg=zeros(1, GROUP_LENGTH);
fol_pos_yg=zeros(1, GROUP_LENGTH);
fol_pos_xg(1)=0;
fol_pos_yg(1)=100;

%从机速度
fol_vel=zeros(1,GROUP_LENGTH);
fol_vel_xg=zeros(1, GROUP_LENGTH);
fol_vel_yg=zeros(1, GROUP_LENGTH);
fol_vel_xg(1)=10.0;
fol_vel_yg(1)=10.0;

%从机角度（偏航角）
fol_Psi=zeros(1, GROUP_LENGTH);
%TODO:计算偏航角初始PSi
fol_Psi(1)=pi/4;

%从机角速度（偏航角速度）
dot_fol_Psi=zeros(1, GROUP_LENGTH);

%从机误差
err_pos_xk=zeros(1, GROUP_LENGTH);
err_pos_yk=zeros(1, GROUP_LENGTH);

err_vel_xk=zeros(1, GROUP_LENGTH);
err_vel_yk=zeros(1, GROUP_LENGTH);
err_vel_k=zeros(1, GROUP_LENGTH);

err_mix_xg=zeros(1, GROUP_LENGTH);%航迹系的x轴方向的混合误差

fol_eta=zeros(1, GROUP_LENGTH);

err_mix_yg=zeros(1, GROUP_LENGTH);%航迹系的y轴方向的混合误差

%===
%===从机控制器参数
%===

k_velmix_pos=0.8;%机体前向误差-位置混合系数
k_velmix_vel=0.8;%机体前向误差-速度混合系数

k_anglemix_pos=0.01;%机体侧向误差-位置混合系数
k_anglemix_eta=3;%机体侧向误差-角度混合系数

Psi_kp=0.5;
Psi_ki=0.0;
Psi_kd=0.1;

%==
%==从机控制器期望值
%==

v_sp_inc=zeros(1, GROUP_LENGTH);
v_sp=zeros(1, GROUP_LENGTH);

%值得注意的是：此处的Psi是偏航角，本次仿真得出的也是控制量是偏航角速度
dot_fol_Psi_sp_inc=zeros(1, GROUP_LENGTH);
dot_fol_Psi_sp=zeros(1, GROUP_LENGTH);

%===
%===仿真主循环
%===

for now=0.0:d_t:end_time
    
time_stamp=time_stamp+1;
TIME(time_stamp)=now;


%===
%位置误差

fol_vel_2d=[fol_vel_xg(time_stamp),fol_vel_yg(time_stamp)];
fol_vel_unit=fol_vel_2d/(norm(fol_vel_2d));
fol_vel(time_stamp)=norm(fol_vel_2d);


led_pos_2d=[led_pos_xg(time_stamp),led_pos_yg(time_stamp)];
fol_pos_2d=[fol_pos_xg(time_stamp),fol_pos_yg(time_stamp)];
err_pos_2d=led_pos_2d-fol_pos_2d;

err_pos_xk(time_stamp)=vec2dot(fol_vel_unit,err_pos_2d);
err_pos_yk(time_stamp)=vec2cross(fol_vel_unit,err_pos_2d);

%速度误差
led_vel_2d=[led_vel_xg(time_stamp),led_vel_yg(time_stamp)];
err_vel_2d=led_vel_2d-fol_vel_2d;


err_vel_xk(time_stamp)=vec2dot(fol_vel_unit,err_vel_2d);
err_vel_yk(time_stamp)=vec2cross(fol_vel_unit,err_vel_2d);

err_vel_k(time_stamp)=sqrt(err_vel_xk(time_stamp)^2+err_vel_yk(time_stamp)^2);

%领机速度在从机航迹系中的投影，计算误差角
led_vel_xk(time_stamp)=vec2dot(fol_vel_unit,led_vel_2d);
led_vel_yk(time_stamp)=vec2cross(fol_vel_unit,led_vel_2d);

%速度角度误差
if (led_vel_xk(time_stamp)==0)&&(led_vel_yk(time_stamp)>0)%正右方
    fol_eta(time_stamp)=pi/2;
elseif (led_vel_xk(time_stamp)==0&&led_vel_yk(time_stamp)<0)%正左方
    fol_eta(time_stamp)=-pi/2;
elseif (led_vel_xk(time_stamp)>=0&&led_vel_yk(time_stamp)==0)%正前方或者重合，为了避免奇怪的控制量
    fol_eta(time_stamp)=0;
elseif(led_vel_xk(time_stamp)<0&&led_vel_yk(time_stamp)==0)%正后方
    fol_eta(time_stamp)=pi;

elseif(led_vel_xk(time_stamp)>0&&led_vel_yk(time_stamp)>0)%右前方
    fol_eta(time_stamp)=atan(led_vel_yk(time_stamp)/led_vel_xk(time_stamp));
elseif(led_vel_xk(time_stamp)>0&&led_vel_yk(time_stamp)<0)%左前方
    fol_eta(time_stamp)=atan(led_vel_yk(time_stamp)/led_vel_xk(time_stamp));
elseif(led_vel_xk(time_stamp)<0&&led_vel_yk(time_stamp)>0)%右后方
    fol_eta(time_stamp)= atan(led_vel_yk(time_stamp)/led_vel_xk(time_stamp))+pi;
elseif(led_vel_xk(time_stamp)<0&&led_vel_yk(time_stamp)<0)%左后方
    fol_eta(time_stamp)= atan(led_vel_yk(time_stamp)/led_vel_xk(time_stamp))-pi;
else
     fol_eta(time_stamp)=0;
end
fol_eta(time_stamp)
%===
%===STEP: 获得机体前向速度期望值
%===

err_mix_xg(time_stamp)=k_velmix_pos*err_pos_xk(time_stamp)+k_velmix_vel*err_vel_k(time_stamp);

%按照情况选定误差，调用增量
[err_prev_xg,err_2prev_xg]=find_err(time_stamp,err_mix_xg);

    
v_sp_inc(time_stamp) = Incremental_PID(0.5, 0.002, 0.0, err_2prev_xg, err_prev_xg, err_mix_xg(time_stamp));

if(time_stamp==1)
v_sp(time_stamp)=v_sp_inc(time_stamp);
elseif(time_stamp>=2)
v_sp(time_stamp)=v_sp_inc(time_stamp)+v_sp(time_stamp-1);
end

%%限幅设计

v_sp(time_stamp)=constrain(v_sp(time_stamp),8.0,34.0);
%v_sp(time_stamp)=0;%现将速度置为0，不飞了，只是转动
%===
%===STEP: 获得机体侧向向加速度速度期望值
%===
err_mix_yg(time_stamp)=k_anglemix_eta*fol_eta(time_stamp)+k_anglemix_pos*err_pos_yk(time_stamp);

%按照情况选定误差，调用增量
[err_prev_yg,err_2prev_yg]=find_err(time_stamp,err_mix_yg);

dot_fol_Psi_sp_inc(time_stamp) = Incremental_PID(Psi_kp,Psi_ki,Psi_kd, err_2prev_yg, err_prev_yg, err_mix_yg(time_stamp));




if(time_stamp==1)
dot_fol_Psi_sp(time_stamp)=dot_fol_Psi_sp_inc(time_stamp);
elseif(time_stamp>=2)
dot_fol_Psi_sp(time_stamp)=dot_fol_Psi_sp_inc(time_stamp)+dot_fol_Psi_sp(time_stamp-1);
end


%===
%===STEP: 更新从机状态
%===

%认为下一时刻真实的角速度，是上一个时刻的期望角速度延时。
dot_fol_Psi(time_stamp+1)=one_order_low_pass(dot_fol_Psi_sp(time_stamp),dot_fol_Psi(time_stamp),0.1);
%===
%===SUB_STEP: 根据航迹侧向加速度计算地面之中的分量
%===

fol_Psi(time_stamp+1)=fol_Psi(time_stamp)+1/2*d_t*(dot_fol_Psi(time_stamp)+dot_fol_Psi(time_stamp+1));


%===
%===SUB_STEP: 更新从机速度
%===

%TODO:注意，这里直接将本时刻的期望速度当做了下一时刻真是速度
fol_vel(time_stamp+1)=one_order_low_pass(v_sp(time_stamp),fol_vel(time_stamp),0.1);
fol_vel_xg(time_stamp+1)=fol_vel(time_stamp+1)*cos(fol_Psi(time_stamp));
fol_vel_yg(time_stamp+1)=fol_vel(time_stamp+1)*sin(fol_Psi(time_stamp));


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
%==仿真结束之后，对仿真中的数据进长度处理
%==
TIME(time_stamp+1)=TIME(time_stamp)+d_t;

v_sp(time_stamp+1)=v_sp(time_stamp);
v_sp_inc(time_stamp+1)=v_sp_inc(time_stamp);

err_mix_xg(time_stamp+1)=err_mix_xg(time_stamp);
err_pos_xk(time_stamp+1)=err_pos_xk(time_stamp);
err_pos_yk(time_stamp+1)=err_pos_yk(time_stamp);

err_vel_k(time_stamp+1)=err_vel_k(time_stamp);
err_vel_xk(time_stamp+1)=err_vel_xk(time_stamp);
err_vel_yk(time_stamp+1)=err_vel_yk(time_stamp);

dot_fol_Psi_sp(time_stamp+1)=dot_fol_Psi_sp(time_stamp);


fol_eta(time_stamp+1)=fol_eta(time_stamp);


%==
%==绘图
%==
%前向
figure(1);
plot(TIME,fol_vel_xg,'y',TIME,led_vel_xg,'b--',TIME,err_vel_xk,'r-.','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Velocity');
xlabel('time s');
ylabel('velocity m/s');
legend('follower-velocity','leader-velocity','error-velocity');
grid on;

figure(2);
plot(TIME,fol_pos_xg,'y',TIME,led_pos_xg,'b--',TIME,err_pos_xk,'r-.','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('前向位置关系图');
xlabel('time s');
ylabel('position m');
legend('follower-position','leader-position','error-position');
grid on;

%侧向
figure(3);
plot(TIME,fol_eta*180/pi,'y','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Angle(degree)');
xlabel('time s');
ylabel('angle degree');
legend('follower-eta');
grid on;

figure(4);
plot(TIME,err_pos_yk,'r-.','LineWidth',2);
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('侧向位置关系图');
xlabel('time s');
ylabel('position m');
legend('err-pos-yk');
grid on;

figure(5);
plot(fol_pos_yg,fol_pos_xg,'y',led_pos_yg,led_pos_xg,'b--','LineWidth',2);
axis equal;
set(gca,'linewidth',1,'fontsize',18,'fontname','Times');
title('Position');
xlabel('E-m');
ylabel('N-m');
legend('follower','leader');
grid on;