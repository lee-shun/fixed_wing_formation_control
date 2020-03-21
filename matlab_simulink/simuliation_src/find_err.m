function [err_prev,err_2prev] = find_err(time_stamp,err)
%FIND_ERR 此处显示有关此函数的摘要
%   此处显示详细说明
    if (time_stamp == 1)%第一次
        err_prev = 0;
        err_2prev = 0;
    elseif (time_stamp == 2)%第二次
        err_prev = err(time_stamp - 1);
        err_2prev = 0;
    elseif (time_stamp >= 3)%后面正常计算
        err_prev = err(time_stamp - 1);
        err_2prev = err(time_stamp - 2);
    end
end

