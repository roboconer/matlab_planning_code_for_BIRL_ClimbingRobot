function [ v ] = UpdateV( particle,velocity,pbest,gbest,index,dimen,iter,velocity_min,velocity_max,c1,c2,w )
%UPDATEV Summary of this function goes here
%   Detailed explanation goes here
r1=rand;
r2=rand;
v=w((index))*velocity(index,dimen,iter)+c1(index)*r1*(pbest(index,dimen,iter)-particle(index,dimen,iter))+c2(index)*r2*(gbest(1,dimen,iter)-particle(index,dimen,iter));
%�����ٶ�
if v>velocity_max
   v=velocity_max;%��(0.3e-10)��ֹ�ٶȹ̶����ϱ߽�
end
if v<velocity_min
   v=velocity_min;%��(0.7e-10)��ֹ�ٶȹ̶����±߽�
end
end