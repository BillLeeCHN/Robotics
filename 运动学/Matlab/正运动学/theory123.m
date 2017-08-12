function theory123( )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

syms a1 a2 a3 a4 a5 a6
T01 = [-sin(a1), -cos(a1), 0, 0; cos(a1), -sin(a1), 0, 0; 0, 0, 1, 320; 0, 0, 0, 1];
T12= [-sin(a2), -cos(a2), 0, 100; 0, 0, -1, 0; cos(a2), -sin(a2), 0, 0; 0, 0, 0, 1];
T23= [cos(a3), -sin(a3), 0, 400; sin(a3), cos(a3), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
T34 = [cos(a4), sin(a4), 0, 0; 0, 0, -1, -400; -sin(a4), cos(a4), 0, 0; 0, 0, 0, 1];
T45 = [cos(a5), -sin(a5), 0, 0; 0, 0, 1, 0; -sin(a5), -cos(a5), 0, 0; 0, 0, 0, 1];
T56 = [cos(a6), sin(a6), 0, 0; 0, 0, -1, 0; -sin(a6), cos(a6), 0, 0; 0, 0, 0, 1];

syms nx ox ax px ny oy ay py nz oz az pz
T6 = [nx, ox, ax, px; ny, oy, ay, py; nz, oz, az, pz; 0, 0, 0, 1];
Tleft = T01 \ T6;
% Tleft = ...
% [  (ny*cos(a1) - nx*sin(a1))/(cos(a1)^2 + sin(a1)^2),  (oy*cos(a1) - ox*sin(a1))/(cos(a1)^2 + sin(a1)^2),  (ay*cos(a1) - ax*sin(a1))/(cos(a1)^2 + sin(a1)^2),  (py*cos(a1) - px*sin(a1))/(cos(a1)^2 + sin(a1)^2);
%   -(nx*cos(a1) + ny*sin(a1))/(cos(a1)^2 + sin(a1)^2), -(ox*cos(a1) + oy*sin(a1))/(cos(a1)^2 + sin(a1)^2), -(ax*cos(a1) + ay*sin(a1))/(cos(a1)^2 + sin(a1)^2), -(px*cos(a1) + py*sin(a1))/(cos(a1)^2 + sin(a1)^2);
%                                                   nz,                                                 oz,                                                 az,                                           pz - 320;
%                                                    0,                                                  0,                                                  0,                                                  1];
%  

T16 = T12 * T23 * T34 * T45 * T56;
Tright = T16;

% Tright = ...
% [ cos(a6)*(sin(a5)*(sin(a2)*sin(a3) - cos(a2)*cos(a3)) - cos(a4)*cos(a5)*(cos(a2)*sin(a3) + cos(a3)*sin(a2))) + sin(a4)*sin(a6)*(cos(a2)*sin(a3) + cos(a3)*sin(a2)),   sin(a6)*(sin(a5)*(sin(a2)*sin(a3) - cos(a2)*cos(a3)) - cos(a4)*cos(a5)*(cos(a2)*sin(a3) + cos(a3)*sin(a2))) - cos(a6)*sin(a4)*(cos(a2)*sin(a3) + cos(a3)*sin(a2)), - cos(a5)*(sin(a2)*sin(a3) - cos(a2)*cos(a3)) - cos(a4)*sin(a5)*(cos(a2)*sin(a3) + cos(a3)*sin(a2)), 400*cos(a2)*cos(a3) - 400*sin(a2)*sin(a3) - 400*sin(a2) + 100;
%                                                                                                                           cos(a4)*sin(a6) + cos(a5)*cos(a6)*sin(a4),                                                                                                                           cos(a5)*sin(a4)*sin(a6) - cos(a4)*cos(a6),                                                                                     sin(a4)*sin(a5),                                                             0;
%   sin(a4)*sin(a6)*(sin(a2)*sin(a3) - cos(a2)*cos(a3)) - cos(a6)*(sin(a5)*(cos(a2)*sin(a3) + cos(a3)*sin(a2)) + cos(a4)*cos(a5)*(sin(a2)*sin(a3) - cos(a2)*cos(a3))), - sin(a6)*(sin(a5)*(cos(a2)*sin(a3) + cos(a3)*sin(a2)) + cos(a4)*cos(a5)*(sin(a2)*sin(a3) - cos(a2)*cos(a3))) - cos(a6)*sin(a4)*(sin(a2)*sin(a3) - cos(a2)*cos(a3)),   cos(a5)*(cos(a2)*sin(a3) + cos(a3)*sin(a2)) - cos(a4)*sin(a5)*(sin(a2)*sin(a3) - cos(a2)*cos(a3)),       400*cos(a2) + 400*cos(a2)*sin(a3) + 400*cos(a3)*sin(a2);
%                                                                                                                                                                   0,                                                                                                                                                                   0,                                                                                                   0,                                                             1];
%  
Tleft
Tright
end

