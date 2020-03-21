function [increment] = Incremental_PID(kp, ki, kd, Prev2Err, PrevErr, Err)
    %UNTITLED 此处显示有关此函数的摘要
    %本函数是增量式PID程序，输入值为上上一次误差，上一次误差以及本次误差
    %   此处显示详细说明

    param_p = kp * (Err - PrevErr);

    param_i = ki * Err;

    param_d = kd * (Err - 2 * PrevErr + Prev2Err);

    increment = param_p + param_i + param_d;

end
