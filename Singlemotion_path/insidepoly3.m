function [state] = insidepoly3(point,poly)        %判断点是否在空间多边形内部
num_p=size(poly,1);
mean_p=zeros(1,3);
kk=0;
for i=1:num_p
    mean_p = mean_p + poly(i,:);
end
mean_p = mean_p/num_p;
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
vec = vec/norm(vec);
v2 = (point-poly(1,:))/norm(point-poly(1,:));
% vec2 = cross(poly(1,:)-poly(3,:),poly(2,:)-poly(1,:));
% vec = (vec +vec2)/2;
if abs(dot(v2,vec))>2e-3
    state = 0;
    return;
end

for i=1:num_p
    ls = poly(i,:);
    if i==num_p
    le = poly(1,:);
    else
        le = poly(i+1,:);
    end
    if (dot(cross(point-ls,le-ls),cross(mean_p-ls,le-ls))<0)
         kk=1;
        break;
    end
end
if kk==0
    state =1;
else
    state=0;
end
end