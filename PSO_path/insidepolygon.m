function [ state ] = insidepolygon(polygon,point)
%����Ƿ���͹������ڲ������ڲ�Ϊ1�������ڲ�����0
%   �˴���ʾ��ϸ˵��
num_p=size(polygon,1);
mean_p=zeros(1,2);
kk=0;
for i=1:num_p
mean_p = mean_p + polygon(i,:);
end
mean_p = mean_p/num_p;
for i=1:num_p
    ls = polygon(i,:);
    if i==num_p
    le = polygon(1,:);
    else
        le = polygon(i+1,:);
    end
    if ((cross2v(point-ls,le-ls))*(cross2v(mean_p-ls,le-ls))<0)
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
%%
function [cro] =cross2v(v1,v2)      %��ά�����Ĳ��
cro=v1(1,1)*v2(1,2)-v1(1,2)*v2(1,1);
end