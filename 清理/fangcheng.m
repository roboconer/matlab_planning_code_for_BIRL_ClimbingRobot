function [c,d] = fangcheng(a1 ,b1)
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明

syms x y;
s1=a1*x-b1*y-8;
s2=a1*x+b1*y-19;
s=solve(s1,s2);
c=s.x;
d=s.y;
end

