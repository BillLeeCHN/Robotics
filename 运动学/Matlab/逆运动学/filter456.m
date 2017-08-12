function [ angle_456 ] = filter456( angle_456, a456_currentdegree )
% function:1��ȥ��������4,5,6�����ؽڲ����˶���Χ�ڵĽ� 2��ѡ��ؽ��˶�������С�ĽǶ�ֵ
% input: 2��⣬��ǰ�����ؽڵĽǶ�
% output:���ŵ�һ��3�ؽڽǶ�

%--------------------------------------------------------------------------
% 1��ȥ��������ǰ�����ؽڲ����˶���Χ�ڵĽ�
%--------------------------------------------------------------------------
% ��þ���angle_456�ж�����
[count,count2] = size(angle_456);

while count >= 1
    if angle_456(count,1) <= -200 || angle_456(count,1) >= 200 || angle_456(count,2) <= -135 || angle_456(count,2) >= 135 ...
         || angle_456(count,3) <= -360 || angle_456(count,3) >= 360
    angle_456(count,:) = [];
    end
    
    count = count - 1;
end
% count��ֵ���Ǻ��ж�����ֵ�ڹؽ��˶���Χ֮�ڡ�
[count, count2] = size(angle_456);
%--------------------------------------------------------------------------
% 2��ѡ��ؽ��˶�������С�ĽǶ�ֵ��a456_currentdegree
%--------------------------------------------------------------------------
while count >= 2
    % �ж�a4�Ƕ�
    if abs(angle_456(1,1)-a456_currentdegree(1)) < abs(angle_456(2,1)-a456_currentdegree(1))
        angle_456(2,:) = [];
    elseif abs(angle_456(1,1)-a456_currentdegree(1)) > abs(angle_456(2,1)-a456_currentdegree(1))
        angle_456(1,:) = [];
    else
        % a4 �Ƕ���ͬ����Ҫ�ж�a5
        if abs(angle_456(1,2)-a456_currentdegree(2)) < abs(angle_456(2,2)-a456_currentdegree(2))
            angle_456(2,:) = [];
        elseif abs(angle_456(1,2)-a456_currentdegree(2)) > abs(angle_456(2,2)-a456_currentdegree(2))
            angle_456(1,:) = [];
        else
            % a4,a5 �Ƕ���ͬ����Ҫ�ж�a6
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

