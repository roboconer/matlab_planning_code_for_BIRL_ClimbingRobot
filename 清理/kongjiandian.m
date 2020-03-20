function [ pp ] = kongjiandian( l1,l2,l3 )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
p1 = [0 0 0];
p2 = [60 0 0];
p3 = [0 -60 0];
syms x y z;
s1 = x^2+y^2+z^2-l1^2;
s2 = (x-p2(1))^2+y^2+z^2-l2^2;
s3 = x^2+(y-p3(2))^2+z^2-l3^2;
%[x,y,z] = solve('x^2+y^2+z^2=l1','(x-60)^2+y^2+z^2=l2','x^2+(y-60)^2+z^2=l3');
s=solve(s1,s2,s3);
pp=vpa([s.x,s.y,s.z]/100);

end

