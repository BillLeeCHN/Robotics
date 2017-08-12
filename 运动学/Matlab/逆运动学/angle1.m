function [ a11_degree,a12_degree ] = angle1( px, py )
% function:��ùؽ�һ�ĽǶ�
% input: ��6����ϵ��λ�ã�px,py, ��λ��mm
% output:�ؽ�һ�ĽǶȣ�a11_degree, a12_degree, ��λ����

a1 = atan2(-px, py)
a1_degree = a1 * 180 / pi;
%--------------------
% a1������ֵ��a11,a12
% a11Ϊ��ֵ��a12Ϊ��ֵ
if a1_degree > 0
    a11_degree = a1_degree;
    a12_degree = a11_degree - 180;
else
    a12_degree = a1_degree;
    a11_degree = 180 + a12_degree;
end

end

