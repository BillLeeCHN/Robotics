function [ world06 ] = WorldCoordinate6( world07 )
% function:����7����ϵ��λ��ת���ɵ�6����ϵ��λ��
% input: world07��1 x 6 �ľ��󣬻����˵�7����ϵ��λ������������ϵ�еı�ʾ,X,Y,Z,U,V,W
% output:world06��1 x 6 �ľ��󣬻����˵�6����ϵ��λ������������ϵ�еı�ʾ,X,Y,Z,U,V,W

syms zangle yangle xangle % U V W

% ��ʽ��X-Y-Z�̶�������ϵ
% Rx = [1, 0, 0; 0, cos(xangle), -sin(xangle); 0, sin(xangle), cos(xangle)];
% Ry = [cos(yangle), 0, sin(yangle); 0, 1, 0; -sin(yangle), 0, cos(yangle)];
% Rz = [cos(zangle), -sin(zangle), 0; sin(zangle), cos(zangle), 0; 0, 0, 1];
% Rxyz = Rz * Ry * Rx;

% �������3 x 3
% Rxyz = ...
% [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle);
%   cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle);
%              -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle)];
      
% ����4 x 4
% Rxyz -> Txyz
Txyz =@(px, py, pz,zangle,yangle,xangle) ...
    [ cos(yangle)*cos(zangle), cos(zangle)*sin(xangle)*sin(yangle) - cos(xangle)*sin(zangle), sin(xangle)*sin(zangle) + cos(xangle)*cos(zangle)*sin(yangle), px;
      cos(yangle)*sin(zangle), cos(xangle)*cos(zangle) + sin(xangle)*sin(yangle)*sin(zangle), cos(xangle)*sin(yangle)*sin(zangle) - cos(zangle)*sin(xangle), py;
                 -sin(yangle),                                       cos(yangle)*sin(xangle),                                       cos(xangle)*cos(yangle), pz;
                            0,                                                             0,                                                             0, 1];
% ��������������ϵ����
px = world07(1);                % X
py = world07(2);                % Y
pz = world07(3);                % Z
zangle = world07(4) * pi / 180; % U
yangle = world07(5) * pi / 180; % V
xangle = world07(6) * pi / 180; % W

% 6->7����α任����
T67 = [1, 0, 0, 0; 
       0, 1, 0, 0; 
       0, 0, 1, 65; % 65���ǵ�6����ϵ����7����ϵ��Z���ƶ����루��λ��mm��
       0, 0, 0, 1]; 

% ��6����ϵ����������ϵ�е�λ��
T06 =  Txyz(px, py, pz,zangle,yangle,xangle) / T67;

% ��6����ϵ����ڵ�7����ϵ������λ�÷����˱仯����̬��û�б仯��
world06 = [T06(1,4),T06(2,4),T06(3,4),world07(4),world07(5),world07(6)];
end

