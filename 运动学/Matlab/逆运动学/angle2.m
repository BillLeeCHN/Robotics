function [ a2_degree ] = angle2( m, n, a3 )
% function:获得关节二的角度
% input: 三个参数
% output:关节二的角度，单位：度

a2 = acos(n * cos(a3)/(2*(sin(a3)+1)) + m/2);
a2_degree = a2 *180 / pi;

%--------------------------
% 验证a2的正负号：
flag = m * cos(a3) / (1 + sin(a3)) - n;
if flag > 0
    a2_degree = abs(a2_degree);
else
    a2_degree = -abs(a2_degree);
end
%--------------------------

end

