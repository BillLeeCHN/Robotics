function [ a11_degree,a12_degree ] = angle1( px, py )
% function:获得关节一的角度
% input: 第6坐标系的位置，px,py, 单位：mm
% output:关节一的角度，a11_degree, a12_degree, 单位：度

a1 = atan2(-px, py)
a1_degree = a1 * 180 / pi;
%--------------------
% a1有两个值：a11,a12
% a11为正值，a12为负值
if a1_degree > 0
    a11_degree = a1_degree;
    a12_degree = a11_degree - 180;
else
    a12_degree = a1_degree;
    a11_degree = 180 + a12_degree;
end

end

