function [ a31_degree, a32_degree ] = angle3( m, n )
% function:获得关节三的角度
% input: 两个参数
% output:关节三的角度，单位：度

a3  = asin((m^2 + n^2)/2 - 1);
a3_degree = a3 * 180 / pi;
% 每一个a1的值对应a3有两个值：a31_degree,a32_degree
a31_degree = a3_degree;
a32_degree = 180 - a31_degree;
end

