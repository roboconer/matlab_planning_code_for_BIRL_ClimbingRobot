function [  ] = circle( x,y,r )
%UNTITLED 此处显示有关此函数的摘要
%   输入圆心，半径画圆
rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'LineWidth',2,'Edgecolor','k');

end

