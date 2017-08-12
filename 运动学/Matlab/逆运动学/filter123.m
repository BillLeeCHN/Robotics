function [ angle_123 ] = filter123( angle_123, a123_currentdegree )
% function:1��ȥ��������ǰ�����ؽڲ����˶���Χ�ڵĽ� 2��ѡ��ؽ��˶�������С�ĽǶ�ֵ
% input: 4��⣬��ǰ�����ؽڵĽǶ�
% output:���ŵ�һ��3�ؽڽǶ�

%--------------------------------------------------------------------------
% 1��ȥ��������ǰ�����ؽڲ����˶���Χ�ڵĽ�
%--------------------------------------------------------------------------
% ��þ���angle_123�ж�����
[count,count2] = size(angle_123);

while count >= 1
    if angle_123(count,1) <= -170 || angle_123(count,1) >= 170 || angle_123(count,2) <= -160 || angle_123(count,2) >= 65 ...
         || angle_123(count,3) <= -51 || angle_123(count,3) >= 225
    angle_123(count,:) = [];
    end
    
    count = count - 1;
end
% count��ֵ���Ǻ��ж�����ֵ�ڹؽ��˶���Χ֮�ڡ�
[count, count2] = size(angle_123);

%--------------------------------------------------------------------------
% 2��ѡ��ؽ��˶�������С�ĽǶ�ֵ��a123_currentdegree
%--------------------------------------------------------------------------
while count >= 2
    % �ж�a1�Ƕ�
    if abs(angle_123(1,1)-a123_currentdegree(1)) < abs(angle_123(2,1)-a123_currentdegree(1))
        angle_123(2,:) = [];
    elseif abs(angle_123(1,1)-a123_currentdegree(1)) > abs(angle_123(2,1)-a123_currentdegree(1))
        angle_123(1,:) = [];
    else
        % a1 �Ƕ���ͬ����Ҫ�ж�a2
        if abs(angle_123(1,2)-a123_currentdegree(2)) < abs(angle_123(2,2)-a123_currentdegree(2))
            angle_123(2,:) = [];
        elseif abs(angle_123(1,2)-a123_currentdegree(2)) > abs(angle_123(2,2)-a123_currentdegree(2))
            angle_123(1,:) = [];
        else
            % a1,a2 �Ƕ���ͬ����Ҫ�ж�a3
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

