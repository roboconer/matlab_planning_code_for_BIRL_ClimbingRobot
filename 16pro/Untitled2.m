syms x y z;
syms z1 x1 ;
Rz=[cos(z) -sin(z) 0;sin(z) cos(z) 0;0 0 1];
Ry=[cos(y) 0 sin(y);0 1 0;-sin(y) 0 cos(y)];
Rx=[1 0 0;0 cos(x) -sin(x);0 sin(x) cos(x)];
RGB=Rz*Ry*Rx;
Rz1=[cos(z1) -sin(z1) 0;sin(z1) cos(z1) 0;0 0 1];
Rx1=[1 0 0;0 cos(x1) -sin(x1);0 sin(x1) cos(x1)];
Ry1=[1 0 0;0 1 0;0 0 1];
RGA=Rz1*Ry1*Rx1;
RAB=RGB'*RGA;
