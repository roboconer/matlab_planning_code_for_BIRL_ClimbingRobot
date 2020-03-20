function [ f ] = Fitness1( particle,index,iter,particle_size,interval,acr_obs,att_obs,byp_obs,TR,ps,pe)
%FITNESS Summary of this function goes here
%   Detailed explanation goes here
%ps= [1 0.5];               %起点位置
%pe=[4.8 4.8];              %终点位置
num_byp=size(byp_obs,1);
byp1_obs=zeros(num_byp,2); %绕开的障碍物区域
byp2_obs=zeros(num_byp,2); %绕开的障碍物扩展区域
num_acr=size(acr_obs,1);
acr1_obs=zeros(num_acr,2); %跨越的障碍物区域
acr2_obs=zeros(num_acr,2); %跨越的障碍物扩展区域
num_att=size(att_obs,1);
att1_obs=zeros(num_acr,2); %吸附的障碍物区域
att2_obs=zeros(num_acr,2); %吸附的障碍物扩展区域
att3_obs=zeros(num_acr,2); %跨越的障碍物内缩区域
dis_wei=ones(particle_size+1,1);  %两点间求距离的权重

s=0;
ps1=(-TR'*ps')';
k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;

stm=0;
acr_stm1=zeros(2*k_acr,1);
acr_stm2=zeros(2*k_acr,1);
att_stm1=zeros(2*k_att,1);
att_stm2=zeros(3*k_att,1);
byp_stm1=zeros(k_byp,1);
byp_stm2=zeros(k_byp,1);
mami_byp=zeros(k_byp,4);
mami_acr=zeros(k_acr,4);
mami_att=zeros(k_att,4);
pss=[0 0];
pee=(TR'*pe'+ps1')';
for i=1:num_byp                                           %障碍物及其各种区域的转化
    byp1_obs(i,:)=(TR'*byp_obs(i,1:2)'+ps1')';
    byp2_obs(i,:)=(TR'*byp_obs(i,3:4)'+ps1')';
end
for i=1:num_acr
    acr1_obs(i,:)=(TR'*acr_obs(i,1:2)'+ps1')';
    acr2_obs(i,:)=(TR'*acr_obs(i,3:4)'+ps1')';
end
for i=1:num_att
    att1_obs(i,:)=(TR'*att_obs(i,1:2)'+ps1')';
    att2_obs(i,:)=(TR'*att_obs(i,3:4)'+ps1')';
    att3_obs(i,:)=(TR'*att_obs(i,5:6)'+ps1')';
end
 for i=1:k_byp                                                %获取障碍区域的值的变化范围
     mami_byp(:,1)=max(byp2_obs(i*4-3:i*4,1));
     mami_byp(:,2)=min(byp2_obs(i*4-3:i*4,1));
     mami_byp(:,3)=max(byp2_obs(i*4-3:i*4,2));
     mami_byp(:,4)=min(byp2_obs(i*4-3:i*4,2));
 end
 for i=1:k_acr
     mami_acr(:,1)=max(acr2_obs(i*4-3:i*4,1));
     mami_acr(:,2)=min(acr2_obs(i*4-3:i*4,1));
     mami_acr(:,3)=max(acr2_obs(i*4-3:i*4,2));
     mami_acr(:,4)=min(acr2_obs(i*4-3:i*4,2));
 end
 for i=1:k_att
     mami_att(:,1)=max(att2_obs(i*4-3:i*4,1));
     mami_att(:,2)=min(att2_obs(i*4-3:i*4,1));
     mami_att(:,3)=max(att2_obs(i*4-3:i*4,2));
     mami_att(:,4)=min(att2_obs(i*4-3:i*4,2));
 end
for i=1:particle_size+1
    if i>particle_size
        pw=pee;
    else
    pw=[interval*i,particle(index,i,iter)];
    end
    if i==1
        pwlast=[0 0];
    else
        pwlast=[interval*(i-1),particle(index,i-1,iter)];
    end
        pwl=[pw;pwlast];
        
        for j=1:k_byp
              if pw(1)>mami_byp(j,1)||pw(1)<mami_byp(j,2)||pw(2)>mami_byp(j,3)||pw(2)<mami_byp(j,4)
                 byp_stm1(j) = 0;
                 %byp_stm1(k_byp+j) = 0;
             else
            byp_stm1(j) = insidepolygon(byp2_obs(j*4-3:j*4,:),pw);
            %byp_stm1(k_byp+j) = insidepolygon(byp2_obs(j*4-3:j*4,:),pw);
             if byp_stm1(j)==1
                     break;
             end
              end
           %[inter_l1,byp_stm2(j)] =linepolyintersection(pwl,byp1_obs(j*4-3:j*4,:),2);
            [inter_l1,byp_stm2(j)] =linepolyintersection(pwl,byp2_obs(j*4-3:j*4,:),2);
              if byp_stm2(j)>=1
                     break;
             end
         end
         if any(byp_stm1==1)
             dis_wei(i)=50;
             continue;
         elseif any(byp_stm2>=1)
             dis_wei(i)=50;
             continue;
         end
         
         for j=1:k_acr
             if pw(1)>mami_acr(j,1)||pw(1)<mami_acr(j,2)||pw(2)>mami_acr(j,3)||pw(2)<mami_acr(j,4)
                 acr_stm1(j) = 0;
                 acr_stm1(k_acr+j) = 0;
             else
                 acr_stm1(j) = insidepolygon(acr1_obs(j*4-3:j*4,:),pw);
                 acr_stm1(k_acr+j) = insidepolygon(acr2_obs(j*4-3:j*4,:),pw);
                 if acr_stm1(j)==1||acr_stm1(k_acr+j)==1
                     break;
                 end
             end
           [inter_l1,acr_stm2(j)] =linepolyintersection(pwl,acr1_obs(j*4-3:j*4,:),2);
           [inter_l1,acr_stm2(k_acr+j)] =linepolyintersection(pwl,acr2_obs(j*4-3:j*4,:),2); 
            if acr_stm2(j)==2||acr_stm2(k_acr+j)==2
                     break;
                 end
         end
         if any(acr_stm1==1)
             dis_wei(i)=1.2;%1.5
             continue;
         elseif any(acr_stm2(1:k_acr)==2)
             dis_wei(i)=1.1;
             continue;
         elseif  any(acr_stm2(k_acr+1:2*k_acr)==2)
             dis_wei(i)=1.2;%1.3
             continue;
         end
         
         for j=1:k_att
              if pw(1)>mami_att(j,1)||pw(1)<mami_att(j,2)||pw(2)>mami_att(j,3)||pw(2)<mami_att(j,4)
                 att_stm1(j) = 0;
                 att_stm1(k_att+j) = 0;
              else
                 att_stm1(j) = insidepolygon(att3_obs(j*4-3:j*4,:),pw);
                 att_stm1(k_att+j) = insidepolygon(att2_obs(j*4-3:j*4,:),pw);
                 if att_stm1(j)==1||att_stm1(k_att+j)==1
                     break;
                 end
              end
           [inter_l1,att_stm2(j)] =linepolyintersection(pwl,att3_obs(j*4-3:j*4,:),2);
           [inter_l1,att_stm2(k_att+j)] =linepolyintersection(pwl,att1_obs(j*4-3:j*4,:),2);
            [inter_l1,att_stm2(2*k_att+j)] =linepolyintersection(pwl,att2_obs(j*4-3:j*4,:),2);
               if att_stm2(j)==2||att_stm2(k_att+j)==2
                     break;
               end
         end
         if any(att_stm1(1:k_att)==1)
             dis_wei(i)=1;
         elseif any(att_stm1(k_att+1:2*k_att)==1)
             dis_wei(i)=1.2;%1.2
         elseif any(att_stm2(1:k_att)==2)
             dis_wei(i)=1;
         elseif any(att_stm2(k_att+1:2*k_att)==2)
              dis_wei(i)=1.3;%1.5
         elseif any(att_stm2(2*k_att+1:3:k_att)==2)
             dis_wei(i)=1;
         end
         
end
s1=norm([interval particle(index,1,iter)]-[0 0])*dis_wei(1);
s2=norm([interval*particle_size,particle(index,particle_size,iter)]-pee)*dis_wei(particle_size+1);

for i=1:particle_size-1
    s=s+sqrt(interval^2+(particle(index,i+1,iter)-particle(index,i,iter))^2)*dis_wei(i+1);
end

f=s1+s+s2;
end