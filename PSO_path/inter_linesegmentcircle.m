function [ inter1,inter2,num_inter ] = inter_linesegmentcircle( p1,p2,o,r)
% 求线段与圆的交点，有两个交点num_inter返回2，一个交点返回1，没有交点返回0
%   此处显示详细说明
x0=o(1);y0=o(2);
x1=p1(1);y1=p1(2);
x2=p2(1);y2=p2(2);
dx=x2-x1;dy=y2-y1;
A=dx*dx+dy*dy;
B=2*dx*(x1-x0)+2*dy*(y1-y0);
C=(x1-x0)^2+(y1-y0)^2-r*r;
delta=B*B-4*A*C;
if delta>=0
    t1= (-B-sqrt(delta))/(2*A);
    t2= (-B+sqrt(delta))/(2*A);
if t1>=0&&t1<=1&&t2>=0&&t2<=1
    num_inter=2;
    inter1=p1+t1*(p2-p1);
    inter2=p1+t2*(p2-p1);
elseif t1>=0&&t1<=1
     num_inter=1;
    inter1=p1+t1*(p2-p1);
    inter2=p1+t1*(p2-p1);
elseif t2>=0&&t2<=1
     num_inter=1;
    inter1=p1+t2*(p2-p1);
    inter2=p1+t2*(p2-p1);
else
     num_inter=0;
    inter1=[0,0];
    inter2=[0,0];
end
else
    num_inter=0;
    inter1=[0,0];
    inter2=[0,0];
end

end

