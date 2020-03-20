function [ cir_p ] = cir_seem_poly( O,vec,r,num_bor )
%UNTITLED ��֪�ռ�Բ��Բ�ģ��������Ͱ뾶����ռ�Բ�ϵĵ㣬���ݷָ��ܶ���ⲻͬ����Ŀ�Ľ�Բ�����
%   �Ƚ�������ϵ������������ϵ��ת��Բ�ϵĵ�
vec = vec/norm(vec);
R_bz = vec;
R_by = [vec(3),0,-vec(1)]/norm([vec(3),0,-vec(1)]);
R_bx = cross(R_by,R_bz);
R_b = [R_bx',R_by',R_bz'];
inter_ang = 360/num_bor;
cir_p = zeros(num_bor,3);
for i = 1:num_bor
    ang2 = (i-1)*inter_ang;
   ang = deg2rad(ang2);
  % cir_p(i,:) = ([cos(ang),-sin(ang),0;sin(ang),cos(ang),0;0,0,1]*R_b*[0,r,0]')' + O;
  cir_p(i,:) = (R_b*[r*(cos(ang)),r*sin(ang),0]')' + O;
end
%  plot3(cir_p(:,1)*1000,cir_p(:,2)*1000,cir_p(:,3)*1000,'*g','LineWidth',2);
%  axis equal
end

