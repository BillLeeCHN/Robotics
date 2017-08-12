function [ angle_123 ] = filter123( angle_123, a123_currentdegree )
% function:1、去除机器人前三个关节不在运动范围内的角 2、选择关节运动幅度最小的角度值
% input: 4组解，当前三个关节的角度
% output:最优的一组3关节角度

%--------------------------------------------------------------------------
% 1、去除机器人前三个关节不在运动范围内的角
%--------------------------------------------------------------------------
% 获得矩阵angle_123有多少行
[count,count2] = size(angle_123);

while count >= 1
    if angle_123(count,1) <= -170 || angle_123(count,1) >= 170 || angle_123(count,2) <= -160 || angle_123(count,2) >= 65 ...
         || angle_123(count,3) <= -51 || angle_123(count,3) >= 225
    angle_123(count,:) = [];
    end
    
    count = count - 1;
end
% count的值就是含有多少组值在关节运动范围之内。
[count, count2] = size(angle_123);

%--------------------------------------------------------------------------
% 2、选择关节运动幅度最小的角度值，a123_currentdegree
%--------------------------------------------------------------------------
while count >= 2
    % 判断a1角度
    if abs(angle_123(1,1)-a123_currentdegree(1)) < abs(angle_123(2,1)-a123_currentdegree(1))
        angle_123(2,:) = [];
    elseif abs(angle_123(1,1)-a123_currentdegree(1)) > abs(angle_123(2,1)-a123_currentdegree(1))
        angle_123(1,:) = [];
    else
        % a1 角度相同，需要判断a2
        if abs(angle_123(1,2)-a123_currentdegree(2)) < abs(angle_123(2,2)-a123_currentdegree(2))
            angle_123(2,:) = [];
        elseif abs(angle_123(1,2)-a123_currentdegree(2)) > abs(angle_123(2,2)-a123_currentdegree(2))
            angle_123(1,:) = [];
        else
            % a1,a2 角度相同，需要判断a3
            if abs(angle_123(1,3)-a123_currentdegree(3)) < abs(angle_123(2,3)-a123_currentdegree(3))
                angle_123(2,:) = [];
            else
                angle_123(1,:) = [];
            end
        end 
    end
    
    count = count - 1;
end

end

