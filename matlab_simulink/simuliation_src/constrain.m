function [output] = constrain(input,min,max)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
if input>=max
    output=max;
elseif input<=min
    output=min;
else
    output=input;
end
end

