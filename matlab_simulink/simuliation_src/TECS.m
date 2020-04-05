function [theta_sp,thrust_sp,thrust_ingt,theta_ingt] = TECS(height,height_sp,vel,vel_sp,theta,time_stamp,d_t,thrust_ingt,theta_ingt)

GRAVITY_CONSTANT=9.81;

%TECS 此处显示有关此函数的摘要


%1.计算变化率

if (time_stamp==1)
    prev=1;%前一个标志位
else
    prev=time_stamp-1;
end

hgt_rate_setpoint=1/(d_t)*(height_sp(time_stamp)-height_sp(prev));
vel_rate_setpoint=1/(d_t)*(vel_sp(time_stamp)-vel_sp(prev));

hgt_rate=1/(d_t)*(height(time_stamp)-height(prev));
vel_rate=1/(d_t)*(vel(time_stamp)-vel(prev));
vert_vel=vel(time_stamp)*sin(theta(time_stamp));
    

%2. 计算能量值

SPE_setpoint = height_sp(time_stamp) * GRAVITY_CONSTANT;            % potential energy
SKE_setpoint = 0.5 * vel_sp(time_stamp) * vel_sp(time_stamp);       % kinetic energy

% Calculate specific energy rate demands in units of (m**2/sec**3)
SPE_rate_setpoint = hgt_rate_setpoint * GRAVITY_CONSTANT;           % potential energy rate of change
SKE_rate_setpoint = vel_sp(time_stamp) * vel_rate_setpoint;         % kinetic energy rate of change%TODO：此处有错误

% Calculate specific energies in units of (m**2/sec**2)
SPE_estimate = height(time_stamp) * GRAVITY_CONSTANT; % potential energy
SKE_estimate = 0.5 * vel(time_stamp) * vel(time_stamp);	% kinetic energy

% Calculate specific energy rates in units of (m**2/sec**3)
SPE_rate = vert_vel * GRAVITY_CONSTANT; % potential energy rate of changeTODO:此处用哪一个？？
SKE_rate = vel(time_stamp) * vel_rate;	% kinetic energy rate of change


%3. 更新油门

STE_error=(SPE_setpoint+SKE_setpoint)-(SPE_estimate+SKE_estimate);

STE_rate_error=(SPE_rate_setpoint+SKE_rate_setpoint)-(SPE_rate+SKE_rate);

%计算前馈项
throttle_curise=0.2;
k_ff=0.005;

if STE_rate_error>=0
    throttle_ff=throttle_curise+k_ff*STE_rate_error;
    else
        throttle_ff=throttle_curise-k_ff*STE_rate_error;
end

STE_2_throttle=1.0/(20.0);

throttle_sp=(STE_error+STE_rate_error*0.05)*STE_2_throttle;

throttle_sp=constrain(throttle_sp,0.05,0.6);
%计算积分量
integ_thr_max=0.9;
integ_thr_min=0.1;

thrust_ingt(time_stamp+1)=thrust_ingt(time_stamp)+0.05*STE_error*d_t*STE_2_throttle;%这里更新了积分量之后就会在次传出去

if thrust_ingt(time_stamp+1)>integ_thr_max
    thrust_ingt(time_stamp+1)=integ_thr_max;
elseif thrust_ingt(time_stamp+1)<integ_thr_min
    thrust_ingt(time_stamp+1)=integ_thr_min;
end

thrust_sp=throttle_sp+thrust_ingt(time_stamp);

if 

thrust_sp=constrain(thrust_sp,0.1,0.9);


%4.更新俯仰角

SEB_setpoint=SPE_setpoint-SKE_setpoint;
SEB_rate_setpoint=SPE_rate_setpoint-SKE_rate_setpoint;

%Calculate the specific energy balance and balance rate error
SEB_error=SEB_setpoint-(SPE_estimate - SKE_estimate );
SEB_rate_error = SEB_rate_setpoint - (SPE_rate  - SKE_rate );

climb_angle_to_SEB_rate=vel(time_stamp) * 2.0 * GRAVITY_CONSTANT;

theta_ingt(time_stamp+1)=theta_ingt(time_stamp)+SEB_error*d_t*0.05;

SEB_correction = SEB_error - SEB_rate_error * 0.1 + SEB_rate_setpoint * 5.0;

theta_sp = (SEB_correction + theta_ingt(time_stamp)) / climb_angle_to_SEB_rate;
theta_sp=constrain(theta_sp,-pi/6,pi/6);

end

