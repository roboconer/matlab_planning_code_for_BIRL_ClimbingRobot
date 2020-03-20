function [ p ] = UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
%UPDATEP Summary of this function goes here
%   Detailed explanation goes here
p=particle(index,dimen,iter)+velocity(index,dimen,(iter+1));
%限制位置
if p>particle_max(dimen)
   p=particle_max(dimen);%减(0.3e-10)防止位置固定在上边界
end
if p<particle_min(dimen)
   p=particle_min(dimen);%加(0.7e-10)防止位置固定在下边界
end
end