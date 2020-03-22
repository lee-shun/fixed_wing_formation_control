function [output] = one_order_low_pass(input,prev_out,k)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
output=input*k+prev_out*(1-k);
end

