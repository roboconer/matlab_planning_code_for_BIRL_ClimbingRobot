function [ p ] = UpdateP1( particle,velocity,index,dimen,iter,particle_min,particle_max,gbest)
%UPDATEP Summary of this function goes here
%   Detailed explanation goes here
% num_byp=size(byp_obs,1);
% byp1_obs=zeros(num_byp,2); %障碍物区域
% byp2_obs=zeros(num_byp,2); %落足障碍区域
% ps1=(-TR'*ps')';
% con=1;
% k_byp=num_byp/4;
% sta1=zeros(1,k_byp);
% sta2=zeros(1,k_byp);
% for i=1:num_byp
%     byp1_obs(i,:)=(TR'*byp_obs(i,1:2)'+ps1')';
%     byp2_obs(i,:)=(TR'*byp_obs(i,3:4)'+ps1')';
% end
num_particle=size(particle,1);
num_iter=size(particle,3);
if num_particle-index<3&&rem(iter,20)<3
    p=gbest(1,dimen,iter)+(num_particle-index+1)*0.1;
elseif num_particle-index<6&&rem(iter,20)<3
    p=gbest(1,dimen,iter)-(num_particle-index-2)*0.1;
else
p=particle(index,dimen,iter)+velocity(index,dimen,(iter+1));
end
% pw=[0.2*dimen,p];
% if dimen==1
%     pwlast=[0 0];
% else
%     pwlast=[0.2*(dimen-1),particle(index,dimen-1,iter)];
% end
% pwl=[pw;pwlast];
% while con==1
%     for i=1:k_byp
%      sta1(i) = insidepolygon(byp2_obs(i*4-3:i*4,:),pw);
%      [inter_l1,sta2(i)] =linepolyintersection(pwl,byp1_obs(i*4-3:i*4,:),2);
%     end
%    if any(sta1==1)||any(sta2==2)
%      if index>20
%      p=particle(ceil(rand*20),dimen,iter);
%      else
%          p=(particle_min(dimen)+(particle_max(dimen)-particle_min(dimen))*rand);
%      end
%     pw=[0.2*dimen,p];
%     if dimen==1
%         pwlast=[0 0];
%     else
%         pwlast=[0.2*(dimen-1),particle(index,dimen-1,iter)];
%     end
%     pwl=[pw;pwlast];
%  else
%      con=0;
%   end
% end
%限制位置
if p>particle_max(dimen)
   p=particle_max(dimen)-rand*0.001;%减(0.3e-10)防止位置固定在上边界
end
if p<particle_min(dimen)
   p=particle_min(dimen)+rand*0.001;%加(0.7e-10)防止位置固定在下边界
end
end