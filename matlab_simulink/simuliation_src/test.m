%本脚本是此次仿真的入口
%%%%%%%%%%%%%%%%%%%%%全局变量定义%%%%%%%%%%%%%%%%%%%%%
global desire;
global estimated;
global err;

%%%%%%%%%%%%%%%%%%%%%%变量初始化%%%%%%%%%%%%%%%%%%%%%%

desire = 1;
estimated = zeros(1, 300);
err = zeros(1, 300);
inc = zeros(1, 300);
err_prev = 0;
err_2prev = 0;

count = 3; %TODO:注意，此处开始的位置
%%%%%%%%%%%%%%%%%%%%%%仿真主循环%%%%%%%%%%%%%%%%%%%%%%

while (count <= 150)

    %按照情况选定误差
    err(count) = desire - estimated(count);

    if (count == 1)%第一次
        err_prev = 0;
        err_2prev = 0;
    elseif (count == 2)%第二次
        err_prev = err(count - 1);
        err_2prev = 0;
    elseif (count >= 3)%后面正常计算
        err_prev = err(count - 1);
        err_2prev = err(count - 2);
    end

    %按照情况选定误差，调用增量
    inc(count) = Incremental_PID(0.1, 0.1, 0.0, err_2prev, err_prev, err(count));

    estimated(count + 1) = estimated(count) + inc(count);

    %计数加1
    count = count + 1;
end

plot(estimated);
