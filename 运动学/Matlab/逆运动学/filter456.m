function [ angle_456 ] = filter456( angle_456, a456_currentdegree )
% function:1、去除机器人4,5,6三个关节不在运动范围内的角 2、选择关节运动幅度最小的角度值
% input: 2组解，当前三个关节的角度
% output:最优的一组3关节角度

%--------------------------------------------------------------------------
% 1、去除机器人前三个关节不在运动范围内的角
%--------------------------------------------------------------------------
% 获得矩阵angle_456有多少行
[count,count2] = size(angle_456);

while count >= 1
    if angle_456(count,1) <= -200 || angle_456(count,1) >= 200 || angle_456(count,2) <= -135 || angle_456(count,2) >= 135 ...
         || angle_456(count,3) <= -360 || angle_456(count,3) >= 360
    angle_456(count,:) = [];
    end
    
    count = count - 1;
end
% count的值就是含有多少组值在关节运动范围之内。
[count, count2] = size(angle_456);
%--------------------------------------------------------------------------
% 2、选择关节运动幅度最小的角度值，a456_currentdegree
%--------------------------------------------------------------------------
while count >= 2
    % 判断a4角度
    if abs(angle_456(1,1)-a456_currentdegree(1)) < abs(angle_456(2,1)-a456_currentdegree(1))
        angle_456(2,:) = [];
    elseif abs(angle_456(1,1)-a456_currentdegree(1)) > abs(angle_456(2,1)-a456_currentdegree(1))
        angle_456(1,:) = [];
    else
        % a4 角度相同，需要判断a5
        if abs(angle_456(1,2)-a456_currentdegree(2)) < abs(angle_456(2,2)-a456_currentdegree(2))
            angle_456(2,:) = [];
        elseif abs(angle_456(1,2)-a456_currentdegree(2)) > abs(angle_456(2,2)-a456_currentdegree(2))
            angle_456(1,:) = [];
        else
            % a4,a5 角度相同，需要判断a6
            if abs(angle_456(1,3)-a456_currentdegree(3)) < abs(angle_456(2,3)-a456_currentdegree(3))
                angle_456(2,:) = [];
            else
                angle_456(1,:) = [];
            end
        end 
    end
    
    count = count - 1;
end

end

