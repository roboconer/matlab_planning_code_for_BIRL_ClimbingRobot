function [ z1,y1,m] = shanjiao( z,y,x )
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
syms z1 y1 m;
solve('cosd(y)*cosd(z)*cosd(z1)+cosd(y)*sind(z)*sind(z1)=cosd(m)','cosd(y)*sind(z)*cosd(x1)*cosd(z1)-cosd(y)*cosd(z)*cosd(x1)*sind(z1)-sind(y)*sind(x1)=-sind(m)',' cosd(y)*cosd(z)*sind(x1)*sind(z1)-sind(y)*cosd(x1)-cosd(y)*sind(z)*cosd(z1)*sind(x1)=0');

end

