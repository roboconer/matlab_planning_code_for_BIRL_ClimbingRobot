
%编写m文件
function fun=pingmian(p1,p2,p3)
   %用一维行向量输入三个点的坐标
syms D;
AA=[p1;p2;p3];
if det(AA)==0
    display('三点在一条直线');
else
    Result=inv(AA)*[-D;-D;-D];
end
A=Result(1);B=Result(2);C=Result(3);
A,B,C
