function [interpoint,state] =linesegintersection(l1,l2)      %求两线段的交点
interpoint=zeros(1,2);
state = judge_intersection(l1,l2);
if state ==0
    interpoint=[0,0];
else
    t=cross2v(l1(1,:)-l2(1,:),l2(2,:)-l2(1,:))/cross2v(l1(2,:)-l1(1,:),l2(2,:)-l2(1,:));
    t=abs(t);
    interpoint(1,1)=l1(1,1)+(l1(2,1)-l1(1,1))*t;
    interpoint(1,2)=l1(1,2)+(l1(2,2)-l1(1,2))*t;
end
end
%%
function [state] =judge_intersection(l1,l2)    %判断两线段是否相交
eps=10^-6;
minx1=min(l1(1,1),l1(2,1));
maxx1=max(l1(1,1),l1(2,1));
minx2=min(l2(1,1),l2(2,1));
maxx2=max(l2(1,1),l2(2,1));
miny1=min(l1(1,2),l1(2,2));
maxy1=max(l1(1,2),l1(2,2));
miny2=min(l2(1,2),l2(2,2));
maxy2=max(l2(1,2),l2(2,2));
if((minx1>maxx2+eps)||(minx2>maxx1+eps)||(miny1>maxy2+eps)||(miny2>maxy1+eps))
    state =0;
elseif(cross2v(l2(1,:)-l1(1,:),l1(2,:)-l1(1,:))*cross2v(l2(2,:)-l1(1,:),l1(2,:)-l1(1,:))<=0&&cross2v(l1(1,:)-l2(1,:),l2(2,:)-l2(1,:))*cross2v(l1(2,:)-l2(1,:),l2(2,:)-l2(1,:))<=0)
    state =1;
else
    state =0;
end
end

%%
function [cro] =cross2v(v1,v2)      %二维向量的叉乘
cro=v1(1,1)*v2(1,2)-v1(1,2)*v2(1,1);
end