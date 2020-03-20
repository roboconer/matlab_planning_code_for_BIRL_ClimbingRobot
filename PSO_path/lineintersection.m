function [interpoint,state] =lineintersection(l1,l2) 
%UNTITLED 求直线与线段交点，先判断两直线是否相交，假设l2是线段
%   此处显示详细说明
interpoint=zeros(1,2);
state = judge_relation(l1,l2);
if state ==0
    interpoint=[0,0];
else
   a1= l1(2,2)-l1(1,2);b1=l1(1,1)-l1(2,1);
   c1= l1(2,2)*(-b1)-l1(2,1)*a1;
   a2= l2(2,2)-l2(1,2);b2=l2(1,1)-l2(2,1);
   c2= l2(2,2)*(-b2)-l2(2,1)*a2;
   D=a1*b2-a2*b1;
   interpoint(1,1)=(b1*c2-b2*c1)/D;
   interpoint(1,2)=(c1*a2-c2*a1)/D;
   if dot(interpoint(1,:)-l2(1,:),l2(2,:)-l2(1,:))<0||norm(interpoint(1,:)-l2(1,:))/norm(l2(2,:)-l2(1,:))>1
    state=0;
     interpoint=[0,0];
   end
end
end
%%
function [state] =judge_relation(l1,l2)    %判断两直线是否相交
eps=10^-6;
x1=l1(2,1)-l1(1,1);y1=l1(2,2)-l1(1,2);
x2=l2(2,1)-l2(1,1);y2=l2(2,2)-l2(1,2);
if abs(x1)<eps&&abs(x2)<eps
    state=0;
elseif abs(x1)<eps
    state = 1;
elseif abs(x2)<eps
    state = 1;
elseif abs(y1/x1-y2/x2)<eps
    state=0;
else
    state=1;
end
end


%%
function [cro] =cross2v(v1,v2)      %二维向量的叉乘
cro=v1(1,1)*v2(1,2)-v1(1,2)*v2(1,1);
end
