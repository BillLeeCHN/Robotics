function [ a31_degree, a32_degree ] = angle3( m, n )
% function:��ùؽ����ĽǶ�
% input: ��������
% output:�ؽ����ĽǶȣ���λ����

a3  = asin((m^2 + n^2)/2 - 1);
a3_degree = a3 * 180 / pi;
% ÿһ��a1��ֵ��Ӧa3������ֵ��a31_degree,a32_degree
a31_degree = a3_degree;
a32_degree = 180 - a31_degree;
end

