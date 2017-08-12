function [ a4_degree ] = angle4( a4_value, a4_value2, a5 )


% ���a5_degree�ǳ��ӽ�0
limit = 0.001;
a5_degree = a5 * 180 / pi;
if 0 == a5_degree
    a5_degree = -limit;
elseif a5_degree >= -limit && a5_degree < limit
    a5_degree = sign(a5_degree) * limit;
end
a5 = a5_degree * pi / 180;

% ����a4��ֵ 
a4 = acos(a4_value / sin(a5))
a4_degree = a4 * 180 / pi;
%�ж�a4��������
flag = a4_value2 / sin(a5);
if flag > 0
    a4_degree = -abs(a4_degree);
else
    a4_degree = abs(a4_degree);
end

end

