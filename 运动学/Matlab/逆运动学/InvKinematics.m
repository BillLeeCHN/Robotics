function [ angles ] = InvKinematics( pose7 )
% function:逆运动学求解，得到6个关节角
% input: pose7：1 x 6 的矩阵，机器人第7坐标系的位姿在世界坐标系中的表示,X,Y,Z,U,V,W
% output:angles：1 x 6 的矩阵，6个关节角度，a1,a2,a3,a4,a5,a6

% pose7 = [-79.044, 523.42, 690.191, 33.678, -58.782, -116.325];
% 第6坐标系的位姿在世界坐标系的表示
pose6 = WorldCoordinate6(pose7);
%--------------------------------------------------------------------------
% 已知条件
px     = pose6(1,1);            % X
py     = pose6(1,2);            % Y
pz     = pose6(1,3);            % Z
zangle = pose6(1,4) * pi / 180; % U
yangle = pose6(1,5) * pi / 180; % V
xangle = pose6(1,6) * pi / 180; % W
%--------------------------------------------------------------------------
% 求"关节1"的角度：a1
%--------------------------------------------------------------------------
[a11_degree, a12_degree] = angle1(px, py);
a11 = a11_degree * pi / 180;
a12 = a12_degree * pi / 180;
%--------------------------------------------------------------------------
% "关节1"的角度:结束
%--------------------------------------------------------------------------
% 求"关节3"的角度：a3
%--------------------------------------------------------------------------
% 求"关节3"的角度:a31,a32
%--------------------------------------------------------------------------
m = (pz - 320)/400;
n_1 = (py*cos(a11) -px*sin(a11)- 100) / 400;
[a31_degree, a32_degree] = angle3(m, n_1);
a31 = a31_degree * pi / 180;
a32 = a32_degree * pi / 180;
%--------------------------------------------------------------------------
% 求"关节3"的角度:a33,a34
%--------------------------------------------------------------------------
n_2 = (py*cos(a12) -px*sin(a12)- 100) / 400;
[a33_degree, a34_degree] = angle3(m, n_2);
a33 = a33_degree * pi / 180;
a34 = a34_degree * pi / 180;
%--------------------------------------------------------------------------
% "关节3"的角度:结束
%--------------------------------------------------------------------------
% 求"关节2"的角度:a21
% 条件：m, n_1, a31
%--------------------------------------------------------------------------
a21_degree = angle2(m, n_1, a31);
a21 = a21_degree * pi / 180;
%--------------------------------------------------------------------------
% 求"关节2"的角度:a22
% 条件：m, n_1, a32
%--------------------------------------------------------------------------
a22_degree = angle2(m, n_1, a32);
a22 = a22_degree * pi / 180;
%--------------------------------------------------------------------------
% 求"关节2"的角度:a23
% 条件：m, n_2, a33
%--------------------------------------------------------------------------
a23_degree = angle2(m, n_2, a33);
a23 = a23_degree * pi / 180;
%--------------------------------------------------------------------------
% 求"关节2"的角度:a24
% 条件：m, n_2, a34
%--------------------------------------------------------------------------
a24_degree = angle2(m, n_2, a34);
a24 = a24_degree * pi / 180;
%--------------------------------------------------------------------------
% "关节2"的角度:结束
%--------------------------------------------------------------------------
% 筛选最终的角度的过程
% 得到a1,a2,a3的多解后，结果筛选得到最终的a1,a2,a3的一组解。
%前三个关节角的四种组合(a11,a21,a31)(a11,a22,a32)(a12,a23,a33)(a12,a24,a34)
angle_123 = [a11_degree a21_degree a31_degree; 
             a11_degree a22_degree a32_degree; 
             a12_degree a23_degree a33_degree; 
             a12_degree a24_degree a34_degree];
% %--------------------------------------------------------------------------
% 假设机器人此时关节1的角度为a123_currentdegree
a123_currentdegree = [0, 0, 0];
angle_123 = filter123(angle_123, a123_currentdegree);
% 最终获得的三个关节角
a1 = angle_123(1) * pi / 180;
a2 = angle_123(2) * pi / 180;
a3 = angle_123(3) * pi / 180;


%--------------------------------------------------------------------------
% 求后三个角
%--------------------------------------------------------------------------
% R36 =... 
% [   cos(a4)*cos(a5)*cos(a6) - sin(a4)*sin(a6), cos(a6)*sin(a4) + cos(a4)*cos(a5)*sin(a6),  cos(a4)*sin(a5);
%                               cos(a6)*sin(a5),                           sin(a5)*sin(a6),         -cos(a5);
%   - cos(a4)*sin(a6) - cos(a5)*cos(a6)*sin(a4), cos(a4)*cos(a6) - cos(a5)*sin(a4)*sin(a6), -sin(a4)*sin(a5)];
% 
% R03 = ...
% [   cos(a2)*sin(a1)*sin(a3) + cos(a3)*sin(a1)*sin(a2), cos(a2)*cos(a3)*sin(a1) - sin(a1)*sin(a2)*sin(a3), cos(a1);
%   - cos(a1)*cos(a2)*sin(a3) - cos(a1)*cos(a3)*sin(a2), cos(a1)*sin(a2)*sin(a3) - cos(a1)*cos(a2)*cos(a3), sin(a1);
%                     cos(a2)*cos(a3) - sin(a2)*sin(a3),               - cos(a2)*sin(a3) - cos(a3)*sin(a2),       0];
%                 
% R06 = ...
% [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle);
%   cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle);
%              -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle)];
% R_36 = R03 \ R06;
R_36 = ...  
[ -(cos(a2)*cos(a3)*sin(a1)^2*sin(yangle) - cos(a1)^2*sin(a2)*sin(a3)*sin(yangle) - sin(a1)^2*sin(a2)*sin(a3)*sin(yangle) + cos(a1)^2*cos(a2)*cos(a3)*sin(yangle) + cos(a1)*cos(a2)*sin(a3)*cos(yangle)*sin(zangle) + cos(a1)*cos(a3)*sin(a2)*cos(yangle)*sin(zangle) - cos(a2)*sin(a1)*sin(a3)*cos(yangle)*cos(zangle) - cos(a3)*sin(a1)*sin(a2)*cos(yangle)*cos(zangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2)), -(sin(a1)^2*sin(a2)*sin(a3)*cos(yangle)*sin(xangle) + cos(a1)*cos(a2)*sin(a3)*cos(xangle)*cos(zangle) + cos(a1)*cos(a3)*sin(a2)*cos(xangle)*cos(zangle) + cos(a2)*sin(a1)*sin(a3)*cos(xangle)*sin(zangle) + cos(a3)*sin(a1)*sin(a2)*cos(xangle)*sin(zangle) - cos(a1)^2*cos(a2)*cos(a3)*cos(yangle)*sin(xangle) - cos(a2)*cos(a3)*sin(a1)^2*cos(yangle)*sin(xangle) + cos(a1)^2*sin(a2)*sin(a3)*cos(yangle)*sin(xangle) + cos(a1)*cos(a2)*sin(a3)*sin(xangle)*sin(yangle)*sin(zangle) + cos(a1)*cos(a3)*sin(a2)*sin(xangle)*sin(yangle)*sin(zangle) - cos(a2)*sin(a1)*sin(a3)*cos(zangle)*sin(xangle)*sin(yangle) - cos(a3)*sin(a1)*sin(a2)*cos(zangle)*sin(xangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2)),  (cos(a1)*cos(a2)*sin(a3)*cos(zangle)*sin(xangle) + cos(a1)*cos(a3)*sin(a2)*cos(zangle)*sin(xangle) + cos(a2)*sin(a1)*sin(a3)*sin(xangle)*sin(zangle) + cos(a3)*sin(a1)*sin(a2)*sin(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*cos(a3)*cos(xangle)*cos(yangle) + cos(a2)*cos(a3)*sin(a1)^2*cos(xangle)*cos(yangle) - cos(a1)^2*sin(a2)*sin(a3)*cos(xangle)*cos(yangle) - sin(a1)^2*sin(a2)*sin(a3)*cos(xangle)*cos(yangle) - cos(a1)*cos(a2)*sin(a3)*cos(xangle)*sin(yangle)*sin(zangle) - cos(a1)*cos(a3)*sin(a2)*cos(xangle)*sin(yangle)*sin(zangle) + cos(a2)*sin(a1)*sin(a3)*cos(xangle)*cos(zangle)*sin(yangle) + cos(a3)*sin(a1)*sin(a2)*cos(xangle)*cos(zangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));
   (cos(a1)^2*cos(a2)*sin(a3)*sin(yangle) + cos(a1)^2*cos(a3)*sin(a2)*sin(yangle) + cos(a2)*sin(a1)^2*sin(a3)*sin(yangle) + cos(a3)*sin(a1)^2*sin(a2)*sin(yangle) - cos(a1)*cos(a2)*cos(a3)*cos(yangle)*sin(zangle) + cos(a2)*cos(a3)*sin(a1)*cos(yangle)*cos(zangle) + cos(a1)*sin(a2)*sin(a3)*cos(yangle)*sin(zangle) - sin(a1)*sin(a2)*sin(a3)*cos(yangle)*cos(zangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2)), -(cos(a1)*cos(a2)*cos(a3)*cos(xangle)*cos(zangle) - cos(a1)*sin(a2)*sin(a3)*cos(xangle)*cos(zangle) + cos(a2)*cos(a3)*sin(a1)*cos(xangle)*sin(zangle) - sin(a1)*sin(a2)*sin(a3)*cos(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*sin(a3)*cos(yangle)*sin(xangle) + cos(a1)^2*cos(a3)*sin(a2)*cos(yangle)*sin(xangle) + cos(a2)*sin(a1)^2*sin(a3)*cos(yangle)*sin(xangle) + cos(a3)*sin(a1)^2*sin(a2)*cos(yangle)*sin(xangle) + cos(a1)*cos(a2)*cos(a3)*sin(xangle)*sin(yangle)*sin(zangle) - cos(a2)*cos(a3)*sin(a1)*cos(zangle)*sin(xangle)*sin(yangle) - cos(a1)*sin(a2)*sin(a3)*sin(xangle)*sin(yangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*cos(zangle)*sin(xangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2)), -(cos(a1)*sin(a2)*sin(a3)*cos(zangle)*sin(xangle) - cos(a1)*cos(a2)*cos(a3)*cos(zangle)*sin(xangle) - cos(a2)*cos(a3)*sin(a1)*sin(xangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*sin(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*sin(a3)*cos(xangle)*cos(yangle) + cos(a1)^2*cos(a3)*sin(a2)*cos(xangle)*cos(yangle) + cos(a2)*sin(a1)^2*sin(a3)*cos(xangle)*cos(yangle) + cos(a3)*sin(a1)^2*sin(a2)*cos(xangle)*cos(yangle) + cos(a1)*cos(a2)*cos(a3)*cos(xangle)*sin(yangle)*sin(zangle) - cos(a2)*cos(a3)*sin(a1)*cos(xangle)*cos(zangle)*sin(yangle) - cos(a1)*sin(a2)*sin(a3)*cos(xangle)*sin(yangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*cos(xangle)*cos(zangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));
                                                                                                                                                                                                                                                                                                                                                                                                 (cos(a1)*cos(yangle)*cos(zangle) + sin(a1)*cos(yangle)*sin(zangle))/(cos(a1)^2 + sin(a1)^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (sin(a1)*cos(xangle)*cos(zangle) - cos(a1)*cos(xangle)*sin(zangle) + cos(a1)*cos(zangle)*sin(xangle)*sin(yangle) + sin(a1)*sin(xangle)*sin(yangle)*sin(zangle))/(cos(a1)^2 + sin(a1)^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            (cos(a1)*sin(xangle)*sin(zangle) - sin(a1)*cos(zangle)*sin(xangle) + cos(a1)*cos(xangle)*cos(zangle)*sin(yangle) + sin(a1)*cos(xangle)*sin(yangle)*sin(zangle))/(cos(a1)^2 + sin(a1)^2)];

% R_36
%--------------------------------------------------------------------------
% 求a5
%--------------------------------------------------------------------------
% R36(2,3)的元素与R_36(2,3)的元素相等
a5_value = -(cos(a1)*sin(a2)*sin(a3)*cos(zangle)*sin(xangle) - cos(a1)*cos(a2)*cos(a3)*cos(zangle)*sin(xangle) - cos(a2)*cos(a3)*sin(a1)*sin(xangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*sin(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*sin(a3)*cos(xangle)*cos(yangle) + cos(a1)^2*cos(a3)*sin(a2)*cos(xangle)*cos(yangle) + cos(a2)*sin(a1)^2*sin(a3)*cos(xangle)*cos(yangle) + cos(a3)*sin(a1)^2*sin(a2)*cos(xangle)*cos(yangle) + cos(a1)*cos(a2)*cos(a3)*cos(xangle)*sin(yangle)*sin(zangle) - cos(a2)*cos(a3)*sin(a1)*cos(xangle)*cos(zangle)*sin(yangle) - cos(a1)*sin(a2)*sin(a3)*cos(xangle)*sin(yangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*cos(xangle)*cos(zangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));
[ a51_degree, a52_degree] = angle5(a5_value);
a51 = a51_degree * pi / 180;
a52 = a52_degree * pi / 180;
%--------------------------------------------------------------------------
% 求a41
% 条件：a51
%--------------------------------------------------------------------------
% R36(1,3)的元素与R_36(1,3)的元素相等
a4_value = (cos(a1)*cos(a2)*sin(a3)*cos(zangle)*sin(xangle) + cos(a1)*cos(a3)*sin(a2)*cos(zangle)*sin(xangle) + cos(a2)*sin(a1)*sin(a3)*sin(xangle)*sin(zangle) + cos(a3)*sin(a1)*sin(a2)*sin(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*cos(a3)*cos(xangle)*cos(yangle) + cos(a2)*cos(a3)*sin(a1)^2*cos(xangle)*cos(yangle) - cos(a1)^2*sin(a2)*sin(a3)*cos(xangle)*cos(yangle) - sin(a1)^2*sin(a2)*sin(a3)*cos(xangle)*cos(yangle) - cos(a1)*cos(a2)*sin(a3)*cos(xangle)*sin(yangle)*sin(zangle) - cos(a1)*cos(a3)*sin(a2)*cos(xangle)*sin(yangle)*sin(zangle) + cos(a2)*sin(a1)*sin(a3)*cos(xangle)*cos(zangle)*sin(yangle) + cos(a3)*sin(a1)*sin(a2)*cos(xangle)*cos(zangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));
% a4 = acos(a4_value / sin(a5));
% R36(3,3)的元素与R_36(3,3)的元素相等
a4_value2 = (cos(a1)*sin(xangle)*sin(zangle) - sin(a1)*cos(zangle)*sin(xangle) + cos(a1)*cos(xangle)*cos(zangle)*sin(yangle) + sin(a1)*cos(xangle)*sin(yangle)*sin(zangle))/(cos(a1)^2 + sin(a1)^2);
a41_degree = angle4(a4_value, a4_value2, a51);
a41 =  a41_degree * pi / 180;   
%--------------------------------------------------------------------------
% 求a42
% 条件：a52
%--------------------------------------------------------------------------
a42_degree = angle4(a4_value, a4_value2, a52);
a42 =  a42_degree * pi / 180;   
%--------------------------------------------------------------------------
% 求a61
% 条件：a51
%--------------------------------------------------------------------------
% R36(2,1)的元素与R_36(2,1)的元素相等
a6_value = (cos(a1)^2*cos(a2)*sin(a3)*sin(yangle) + cos(a1)^2*cos(a3)*sin(a2)*sin(yangle) + cos(a2)*sin(a1)^2*sin(a3)*sin(yangle) + cos(a3)*sin(a1)^2*sin(a2)*sin(yangle) - cos(a1)*cos(a2)*cos(a3)*cos(yangle)*sin(zangle) + cos(a2)*cos(a3)*sin(a1)*cos(yangle)*cos(zangle) + cos(a1)*sin(a2)*sin(a3)*cos(yangle)*sin(zangle) - sin(a1)*sin(a2)*sin(a3)*cos(yangle)*cos(zangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));
% a6 = acos(a6_value / sin(a5));
% R36(2,2)的元素与R_36(2,2)的元素相等
a6_value2 = -(cos(a1)*cos(a2)*cos(a3)*cos(xangle)*cos(zangle) - cos(a1)*sin(a2)*sin(a3)*cos(xangle)*cos(zangle) + cos(a2)*cos(a3)*sin(a1)*cos(xangle)*sin(zangle) - sin(a1)*sin(a2)*sin(a3)*cos(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*sin(a3)*cos(yangle)*sin(xangle) + cos(a1)^2*cos(a3)*sin(a2)*cos(yangle)*sin(xangle) + cos(a2)*sin(a1)^2*sin(a3)*cos(yangle)*sin(xangle) + cos(a3)*sin(a1)^2*sin(a2)*cos(yangle)*sin(xangle) + cos(a1)*cos(a2)*cos(a3)*sin(xangle)*sin(yangle)*sin(zangle) - cos(a2)*cos(a3)*sin(a1)*cos(zangle)*sin(xangle)*sin(yangle) - cos(a1)*sin(a2)*sin(a3)*sin(xangle)*sin(yangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*cos(zangle)*sin(xangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));

a61_degree = angle6(a6_value, a6_value2, a51);
a61 = a61_degree * pi / 180;
%--------------------------------------------------------------------------
% 求a62
% 条件：a52
%--------------------------------------------------------------------------
% R36(2,1)的元素与R_36(2,1)的元素相等
% a6_value = (cos(a1)^2*cos(a2)*sin(a3)*sin(yangle) + cos(a1)^2*cos(a3)*sin(a2)*sin(yangle) + cos(a2)*sin(a1)^2*sin(a3)*sin(yangle) + cos(a3)*sin(a1)^2*sin(a2)*sin(yangle) - cos(a1)*cos(a2)*cos(a3)*cos(yangle)*sin(zangle) + cos(a2)*cos(a3)*sin(a1)*cos(yangle)*cos(zangle) + cos(a1)*sin(a2)*sin(a3)*cos(yangle)*sin(zangle) - sin(a1)*sin(a2)*sin(a3)*cos(yangle)*cos(zangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));
% a6 = acos(a6_value / sin(a5));
% a6_degree = a6 * 180 / pi;
% R36(2,2)的元素与R_36(2,2)的元素相等
% a6_value2 = -(cos(a1)*cos(a2)*cos(a3)*cos(xangle)*cos(zangle) - cos(a1)*sin(a2)*sin(a3)*cos(xangle)*cos(zangle) + cos(a2)*cos(a3)*sin(a1)*cos(xangle)*sin(zangle) - sin(a1)*sin(a2)*sin(a3)*cos(xangle)*sin(zangle) + cos(a1)^2*cos(a2)*sin(a3)*cos(yangle)*sin(xangle) + cos(a1)^2*cos(a3)*sin(a2)*cos(yangle)*sin(xangle) + cos(a2)*sin(a1)^2*sin(a3)*cos(yangle)*sin(xangle) + cos(a3)*sin(a1)^2*sin(a2)*cos(yangle)*sin(xangle) + cos(a1)*cos(a2)*cos(a3)*sin(xangle)*sin(yangle)*sin(zangle) - cos(a2)*cos(a3)*sin(a1)*cos(zangle)*sin(xangle)*sin(yangle) - cos(a1)*sin(a2)*sin(a3)*sin(xangle)*sin(yangle)*sin(zangle) + sin(a1)*sin(a2)*sin(a3)*cos(zangle)*sin(xangle)*sin(yangle))/((cos(a1)^2 + sin(a1)^2)*(cos(a2)^2*cos(a3)^2 + cos(a2)^2*sin(a3)^2 + cos(a3)^2*sin(a2)^2 + sin(a2)^2*sin(a3)^2));

a62_degree = angle6(a6_value, a6_value2, a52);
a62 = a62_degree * pi / 180;
%--------------------------------------------------------------------------
% "关节6"的角度:结束
%--------------------------------------------------------------------------
% 筛选最终的角度的过程
% 得到a4,a5,a6的多解后，结果筛选得到最终的a4,a5,a6的一组解
% 后三个关节角的四种组合(a41,a51,a61)(a42,a52,a62)
angle_456 = [a41_degree a51_degree a61_degree; 
             a42_degree a52_degree a62_degree];
a456_currentdegree = [0, 0, 0];
angle_456 = filter456(angle_456, a456_currentdegree);
% 最终获得的三个关节角
a4 = angle_456(1,1) * pi / 180;
a5 = angle_456(1,2) * pi / 180;
a6 = angle_456(1,3) * pi / 180;

% 输出6个角度：a1,a2,a3,a4,a5,a6
% angle_123
% angle_456
angles = [angle_123, angle_456];
end

