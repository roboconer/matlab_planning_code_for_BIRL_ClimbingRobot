function [ p ] = UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
%UPDATEP Summary of this function goes here
%   Detailed explanation goes here
p=particle(index,dimen,iter)+velocity(index,dimen,(iter+1));
%����λ��
if p>particle_max(dimen)
   p=particle_max(dimen);%��(0.3e-10)��ֹλ�ù̶����ϱ߽�
end
if p<particle_min(dimen)
   p=particle_min(dimen);%��(0.7e-10)��ֹλ�ù̶����±߽�
end
end