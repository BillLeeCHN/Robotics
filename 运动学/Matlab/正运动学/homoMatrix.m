function [ result ] = homoMatrix( pose7 )
% Calculate homogeneous transformation matrix: T
% pose7: the 7th frame value represented in the world coordinate system

% syms xangle yangle zangle
% Rx = [1, 0, 0; 0, cos(xangle), -sin(xangle); 0, sin(xangle), cos(xangle)];
% Ry = [cos(yangle), 0, sin(yangle); 0, 1, 0; -sin(yangle), 0, cos(yangle)];
% Rz = [cos(zangle), -sin(zangle), 0; sin(zangle), cos(zangle), 0; 0, 0, 1];
% Rxyz = Rz * Ry * Rx;

% 旋转矩阵
% Rxyz = ...
% [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle);
%   cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle);
%              -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle)];

% 第7坐标系的其次变换矩阵
Txyz =@(xangle, yangle, zangle, px, py, pz) ...
    [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle), px;
 cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle), py;
            -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle), pz;
            0, 0, 0, 1];
% 参数
% px = 52.309;
% py = 438.407;
% pz = 889.265;
% zangle = 172.744 * pi / 180;
% yangle = -54.981 * pi / 180;
% xangle = 83.431 * pi / 180;

px = pose7(1);
py = pose7(2);
pz = pose7(3);
zangle = pose7(4) * pi / 180;
yangle = pose7(5) * pi / 180;
xangle = pose7(6) * pi / 180;

T67 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 65; 0, 0, 0, 1];
result =  Txyz(xangle, yangle, zangle, px, py, pz) / T67; 
end

