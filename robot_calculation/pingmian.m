
%��дm�ļ�
function fun=pingmian(p1,p2,p3)
   %��һά���������������������
syms D;
AA=[p1;p2;p3];
if det(AA)==0
    display('������һ��ֱ��');
else
    Result=inv(AA)*[-D;-D;-D];
end
A=Result(1);B=Result(2);C=Result(3);
A,B,C
