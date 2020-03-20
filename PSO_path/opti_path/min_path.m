function f = min_path( x)
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明
 [ps,pe,interval] = assignvalue;
 num_si =length(x);
dis_pse=norm(ps-pe);       %起点到终点距离
pxy=(pe-ps)/dis_pse;
s=0;
TR=[pxy(1) -pxy(2);pxy(2) pxy(1)]; 
ps1=(-TR'*ps')';
pss=[0 0];
pee=(TR'*pe'+ps1')';
s1=norm([interval x(1)]-pss);
s2=norm([interval*num_si x(num_si)]-pee);
for i=1:num_si-1
    s=s+sqrt(interval^2+(x(i+1)-x(i))^2);
end
f=s1+s+s2;
end

