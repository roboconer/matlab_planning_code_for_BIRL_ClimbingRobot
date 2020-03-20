clear
clc
L(1) = Link([pi/2 0.675 0.26 pi/2 0]);
L(2) = Link([0 0 0.645 0 0]);
L(3) = Link([pi/2 0 0.035 pi/2 0]);
L(4) = Link([pi/2 0.67 0 pi/2 0]);
L(5) = Link([pi/2 0 0 pi/2 0]);
L(6) = Link([0 0.115 0 0 0]);
r6 =SerialLink(L);
r6.name = '6R»úÐµ±Û';
q = [0 pi/2 0 0 pi 0];
T = fkine(r6,q)
%teach(r6)
r6.display;
r6.plot([pi/4 pi/2 pi/8 pi/2 7*pi/6 pi/2])