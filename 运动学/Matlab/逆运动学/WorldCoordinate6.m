function [ world06 ] = WorldCoordinate6( world07 )
% function:将第7坐标系的位姿转换成第6坐标系的位姿
% input: world07：1 x 6 的矩阵，机器人第7坐标系的位姿在世界坐标系中的表示,X,Y,Z,U,V,W
% output:world06：1 x 6 的矩阵，机器人第6坐标系的位姿在世界坐标系中的表示,X,Y,Z,U,V,W

syms zangle yangle xangle % U V W

% 公式：X-Y-Z固定角坐标系
% Rx = [1, 0, 0; 0, cos(xangle), -sin(xangle); 0, sin(xangle), cos(xangle)];
% Ry = [cos(yangle), 0, sin(yangle); 0, 1, 0; -sin(yangle), 0, cos(yangle)];
% Rz = [cos(zangle), -sin(zangle), 0; sin(zangle), cos(zangle), 0; 0, 0, 1];
% Rxyz = Rz * Ry * Rx;

% 结果矩阵：3 x 3
% Rxyz = ...
% [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle);
%   cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle);
%              -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle)];
      
% 矩阵：4 x 4
% Rxyz -> Txyz
Txyz =@(px, py, pz,zangle,yangle,xangle) ...
    [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle), px;
      cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle), py;
                 -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle), pz;
                            0,                                                             0,                                                             0, 1];
% 机器人世界坐标系数据
px = world07(1);                % X
py = world07(2);                % Y
pz = world07(3);                % Z
zangle = world07(4) * pi / 180; % U
yangle = world07(5) * pi / 180; % V
xangle = world07(6) * pi / 180; % W

% 6->7的齐次变换矩阵
T67 = [1, 0, 0, 0; 
       0, 1, 0, 0; 
       0, 0, 1, 65; % 65：是第6坐标系到第7坐标系的Z轴移动距离（单位：mm）
       0, 0, 0, 1]; 

% 第6坐标系在世界坐标系中的位姿
T06 =  Txyz(px, py, pz,zangle,yangle,xangle) / T67;

% 第6坐标系相对于第7坐标系，仅仅位置发生了变化，姿态并没有变化。
world06 = [T06(1,4),T06(2,4),T06(3,4),world07(4),world07(5),world07(6)];
end

