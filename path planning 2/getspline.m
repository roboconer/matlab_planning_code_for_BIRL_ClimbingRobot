 function [A,B,C]=getspline(input)
A=27/2*(input(1)-input(2))/input(3)^3;
B=-9/2*(-4*input(2)+5*input(1))/input(3)^2;
C=9/2*(2*input(1)-input(2))/input(3);
 end