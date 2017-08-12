function [ a61_degree ] = angle6( a6_value, a6_value2, a5 )


% ���a5_degree�ǳ��ӽ�0
limit = 0.001;
a5_degree = a5 * 180 / pi;
if 0 == a5_degree
    a5_degree = -limit;
elseif a5_degree >= -limit && a5_degree < limit
    a5_degree = sign(a5_degree) * limit;
end
a5 = a5_degree * pi / 180;


% ����a6��ֵ
a6 = acos(a6_value / sin(a5))
a6_degree = a6 * 180 / pi;
flag = a6_value2 / sin(a5);
if flag > 0
    a61_degree = abs(a6_degree);
else
    a61_degree = -abs(a6_degree);
end


end

