function [ a2_degree ] = angle2( m, n, a3 )
% function:��ùؽڶ��ĽǶ�
% input: ��������
% output:�ؽڶ��ĽǶȣ���λ����

a2 = acos(n * cos(a3)/(2*(sin(a3)+1)) + m/2);
a2_degree = a2 *180 / pi;

%--------------------------
% ��֤a2�������ţ�
flag = m * cos(a3) / (1 + sin(a3)) - n;
if flag > 0
    a2_degree = abs(a2_degree);
else
    a2_degree = -abs(a2_degree);
end
%--------------------------

end

