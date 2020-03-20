function [ dis_wei ] = transform_map( y,ps,pe,interval )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map;
dis_pse=norm(ps-pe);       %起点到终点距离
pxy=(pe-ps)/dis_pse;
particle_size=ceil(dis_pse/interval-1);            %粒子维数（求解变量个数）
TR=[pxy(1) -pxy(2);pxy(2) pxy(1)];

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
dis_wei=ones(particle_size,1);  %两点间求距离的权重

k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;
stm=0;
ps1=(-TR'*ps')';       %位置向量转换
acr_stm1=zeros(2*k_acr,1);
acr_stm2=zeros(2*k_acr,1);
att_stm1=zeros(2*k_att,1);
att_stm2=zeros(2*k_att,1);
byp_stm1=zeros(2*k_byp,1);
byp_stm2=zeros(2*k_byp,1);
pss=[0 0];
pee=(TR'*pe'+ps1')';   
for i=1:num_acr
    acr1_obs(i,:)=(TR'*acr_obs(i,1:2)'+ps1')';
    acr2_obs(i,:)=(TR'*acr_obs(i,3:4)'+ps1')';
end
for i=1:num_att
    att1_obs(i,:)=(TR'*att_obs(i,1:2)'+ps1')';
    att2_obs(i,:)=(TR'*att_obs(i,3:4)'+ps1')';
    att3_obs(i,:)=(TR'*att_obs(i,5:6)'+ps1')';
end
for i=1:num_byp
    byp1_obs(i,:)=(TR'*byp_obs(i,1:2)'+ps1')';
    byp2_obs(i,:)=(TR'*byp_obs(i,3:4)'+ps1')';
end
for i=1:particle_size
    pw=[interval*i,y(i)];
    if i==1
        pwlast=[0 0];
    else
        pwlast=[interval*(i-1),y(i-1)];
    end
        pwl=[pw;pwlast];
         for j=1:k_acr
            acr_stm1(j) = insidepolygon(acr1_obs(j*4-3:j*4,:),pw);
            acr_stm1(k_acr+j) = insidepolygon(acr2_obs(j*4-3:j*4,:),pw);
           [inter_l1,acr_stm2(j)] =linepolyintersection(pwl,acr1_obs(j*4-3:j*4,:),2);
            [inter_l1,acr_stm2(k_acr+j)] =linepolyintersection(pwl,acr2_obs(j*4-3:j*4,:),2);
         end
         if any(acr_stm1==1)
             dis_wei(i)=1.5;
         elseif any(acr_stm2==2)
             dis_wei(i)=1.2;
         end
         for j=1:k_att
            att_stm1(j) = insidepolygon(att3_obs(j*4-3:j*4,:),pw);
            att_stm1(k_att+j) = insidepolygon(att2_obs(j*4-3:j*4,:),pw);
           [inter_l1,att_stm2(j)] =linepolyintersection(pwl,att3_obs(j*4-3:j*4,:),2);
            [inter_l1,att_stm2(k_att+j)] =linepolyintersection(pwl,att2_obs(j*4-3:j*4,:),2);
         end
         if any(att_stm1(1:k_att)==1)
             dis_wei(i)=1;
         elseif any(att_stm1(k_att+1:2*k_att)==1)
             dis_wei(i)=1.2;
         elseif any(att_stm2(1:k_att)==2)
             dis_wei(i)=1;
         elseif any(att_stm2(k_att+1:2*k_att)==2)
              dis_wei(i)=1.5;
         end
         for j=1:k_byp
            byp_stm1(j) = insidepolygon(byp1_obs(j*4-3:j*4,:),pw);
            byp_stm1(k_byp+j) = insidepolygon(byp2_obs(j*4-3:j*4,:),pw);
           [inter_l1,byp_stm2(j)] =linepolyintersection(pwl,byp1_obs(j*4-3:j*4,:),2);
            [inter_l1,byp_stm2(k_byp+j)] =linepolyintersection(pwl,byp2_obs(j*4-3:j*4,:),2);
         end
         if any(byp_stm1==1)
             dis_wei(i)=20;
         elseif any(byp_stm2==2)
             dis_wei(i)=20;
         end
end
end

