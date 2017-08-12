function [ a51_degree, a52_degree] = angle5( a5_value )

a5 = acos(-a5_value)
a5_degree = a5 * 180 / pi;
%--------------------------
% a5有两个值:a51,a52
a51_degree = a5_degree;
a52_degree = -a5_degree;

end

