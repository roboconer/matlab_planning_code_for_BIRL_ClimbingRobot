function [  ] = plot_step(  )  %画落足点
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
global foot;
foot= footgen(0.06,20,60); 
pathstr =[0 0;2 3];
o = [1 1.2];
r=0.58;
[licipoint,tar]= pathcircleinter(pathstr,o,r) ;
apathline=[o;licipoint];
[foot_str] = footstep_strategy(apathline);
plot(foot_str(:,1),foot_str(:,2),'or','LineWidth',2);
hold on;
plot(pathstr(:,1),pathstr(:,2),'-k','LineWidth',2);
plot(o(1,1),o(1,2),'*g','LineWidth',3);
circle( 1,1.2,0.58 );
circle( 1,1.2,0.4 );
axis equal;
axis([0,2,0,2]);

end
%%
function [licipoint,tar]= pathcircleinter(pathstr,o,r)     %给定步距求全局路径上离目标点最近的交点
num_pa=size(pathstr,1);
minl=0;
minlen=10000;
mipo=zeros(1,2);
licipoint =[0 0];

for i=1:num_pa-1
    lipa=[pathstr(i,:);pathstr(i+1,:)];
    [ inter1,inter2,num_inter ] = inter_linesegmentcircle( lipa(1,:),lipa(2,:),o,r);
    if num_inter>=1
        norm1 = norm(pathstr(i+1,:)-inter1);
        norm2 = norm(pathstr(i+1,:)-inter2);
        tar = pathstr(i+1,:);
    else
        continue;
    end
    if norm1<norm2
        %minl = norm1;
        licipoint = inter1;%mipo = inter1;
    else
        %minl =norm2;
        licipoint = inter2;%mipo = inter2;
    end
%     if minl<minlen
%         minlen = minl;
%         licipoint = mipo;
%     end
end
end
%%
%判断落足点的可行性
function [state_judge ] = footstep_amendment(apathline,foot_hei)
%[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height ); 
global map_bor;global acr_obs;global att_obs;global byp_obs;global hei_obs;
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
byp1_obs=zeros(num_byp,2);  %绕开的障碍区域
att1_obs=zeros(num_att,2);
att2_obs=zeros(num_att,2);
att3_obs=zeros(num_att,2);
acr1_obs=zeros(num_acr,2);
acr2_obs=zeros(num_acr,2);
k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;
acr_stm1=zeros(k_acr,1);
acr_stm21=zeros(k_acr,1);
acr_stm22=zeros(k_acr,1);
acr_stm23=zeros(k_acr,1);
att_stm11=zeros(k_att,1);
att_stm12=zeros(k_att,1);
att_stm21=zeros(k_att,1);
att_stm22=zeros(k_att,1);
att_stm23=zeros(k_att,1);
byp_stm1=zeros(k_byp,1);
byp_stm2=zeros(k_byp,1);
byp1_obs=byp_obs(:,3:4);
byp2_obs=byp_obs(:,5:6);
att1_obs=att_obs(:,1:2);
att2_obs=att_obs(:,3:4);
att3_obs=att_obs(:,5:6);
acr1_obs=acr_obs(:,1:2);
acr2_obs=acr_obs(:,3:4);
acr3_obs=acr_obs(:,5:6);
r_max=0.58;
r_min=0.4;
%判断落足点处于何种障碍区域内部或者跨越何种障碍区域
in_bor = insidepolygon(map_bor,apathline(2,:));
 for j=1:k_byp                                
    byp_stm1(j) = insidepolygon(byp1_obs(j*4-3:j*4,:),apathline(2,:));
     if byp_stm1(j)==1 
             break;
     end
   [inter_l1,byp_stm2(j)] =linepolyintersection(apathline,byp2_obs(j*4-3:j*4,:),2);
      if byp_stm2(j)==2         
             break;
      end
 end
   for j=1:k_acr
            acr_stm1(j) = insidepolygon(acr2_obs(j*4-3:j*4,:),apathline(2,:));
                 if acr_stm1(j)==1                 
                   break;
                 end
           [inter_l1,acr_stm21(j)] =linepolyintersection(apathline,acr1_obs(j*4-3:j*4,:),2);
           [inter_l1,acr_stm22(j)] =linepolyintersection(apathline,acr2_obs(j*4-3:j*4,:),2);
           [inter_l1,acr_stm23(j)] =linepolyintersection(apathline,acr3_obs(j*4-3:j*4,:),2);

    end
       for j=1:k_att             
                 att_stm11(j) = insidepolygon(att3_obs(j*4-3:j*4,:),apathline(2,:));
                 if att_stm11(j)==1
                     break;
                 end
                 att_stm12(j) = insidepolygon(att2_obs(j*4-3:j*4,:),apathline(2,:));
                 if att_stm12(j)==1&&att_stm11(j)==0
                     break;
                 end
           [inter_l1,att_stm21(j)] =linepolyintersection(apathline,att1_obs(j*4-3:j*4,:),2);
               if att_stm21(j)==2
                     break;
               end
            [inter_l1,att_stm22(j)] =linepolyintersection(apathline,att3_obs(j*4-3:j*4,:),2);
               if att_stm22(j)==2
                     break;
               end
               [inter_l1,att_stm23(j)] =linepolyintersection(apathline,att2_obs(j*4-3:j*4,:),2); 
       end
      if in_bor==0||any(byp_stm1==1)||any(byp_stm2==2)||any(acr_stm1==1)||any(att_stm12==1)||any(att_stm21==2)||any(att_stm22==2)
          state_judge=0;
          return;
      end
       
      if any(acr_stm21==2)
          for i=1:k_acr
              if acr_stm21(i)==2
                  hei = hei_obs(k_byp+i);
                  [inter_l1,ak] =linepolyintersection(apathline,acr1_obs(i*4-3:i*4,:),2); 
                  inter_len=norm(inter_l1(1,:)-inter_l1(2,:));
                  if -0.8372*inter_len+0.4309>hei
%                    [inter_l2,ak] =linepolyintersection(apathline,acr2_obs(i*4-3:i*4,:),2);
%                    acr_len=norm(inter_l1(1,:)-inter_l1(2,:));
                   dis1 = min([norm(apathline(1,:)-inter_l1(1,:)),norm(apathline(1,:)-inter_l1(2,:))]);
                   dis2 = min([norm(apathline(2,:)-inter_l1(1,:)),norm(apathline(2,:)-inter_l1(2,:))]);
                   m_wid = min([dis1,dis2]);
                   wid2 = 2*(0.5*norm(apathline(1,:)-apathline(2,:))-m_wid);
                   if -0.8372*wid2+0.4309>hei
                       if (hei-0.4309)/(-0.8372)+r_min>norm(apathline(1,:)-apathline(2,:))
                       state_judge =1;
                       return;
                       else
                       state_judge = 0;
                       end
                   else
                       state_judge = 0;
                       return;
                   end
                  else
                       state_judge =0;
                       return;
                  end
              end
          end
      elseif any(acr_stm22==2)
          for i=1:k_acr
              if acr_stm22(i)==2
                 if acr_stm23(i)<2
                     state_judge = 1;
                     return;
                 else
                     state_judge = 0;
                     return;  
                 end
              end
          end
      elseif any(att_stm11==1)
                  hei = abs(foot_hei(1)-foot_hei(2)); 
                  dis_pa = norm(apathline(1,:)-apathline(2,:));
                  sina = hei/r_max;
                  lcos = r_max*sqrt(1-sina^2);
                  if dis_pa <=lcos
                      state_judge =1;
                      return;
                  else
                      state_judge =0;
                      return;
                  end
      elseif any(att_stm23==2)
             state_judge =1;
             return;
      elseif foot_hei(1)>0
           hei = abs(foot_hei(1)-foot_hei(2)); 
                  dis_pa = norm(apathline(1,:)-apathline(2,:));
                  sina = hei/r_max;
                  lcos = r_max*sqrt(1-sina^2);
                  if dis_pa <=lcos
                      state_judge =1;
                      return;
                  else
                      state_judge =0;
                      return;
                  end
      else
          state_judge =1;
      end
end
%%
function foot = footgen(dis_inter,an_inter,anmax)  
%根据给的距离范围，角度范围及其间距生成落足点向量，即步态策略
r_max = 0.58;
r_min = 0.4;
num_dis = ceil((r_max-r_min)/dis_inter)+1;
num_an = ceil(anmax/an_inter)+1;
num_dis2 = num_dis-1;
anmax2 = anmax-50;
num_an2 = ceil((anmax2)/an_inter)+1;
foot = zeros(num_dis*2*num_an,2);
%foot = zeros(num_dis*2*num_an+num_dis2*2*num_an2,2);
num_for = num_dis*2*num_an;
for i =1: num_dis
    for j=1:num_an
        if i ==num_dis && j ==num_an
        foot(2*((i-1)*num_an+j)-1,:) =[r_min/r_max*cos(anmax*pi/180),r_min/r_max*sin(anmax*pi/180)];
        foot(2*((i-1)*num_an+j),:) =[r_min/r_max*cos(anmax*pi/180),-r_min/r_max*sin(anmax*pi/180)];
        elseif i== num_dis
         foot(2*((i-1)*num_an+j)-1,:) =[r_min/r_max*cos(an_inter*(j-1)*pi/180),r_min/r_max*sin(an_inter*(j-1)*pi/180)];
        foot(2*((i-1)*num_an+j),:) =[r_min/r_max*cos(an_inter*(j-1)*pi/180),-r_min/r_max*sin(an_inter*(j-1)*pi/180)];
        elseif j == num_an
        foot(2*((i-1)*num_an+j)-1,:) =[(r_max-(i-1)*dis_inter)/r_max*cos(anmax*pi/180),(r_max-(i-1)*dis_inter)/r_max*sin(anmax*pi/180)];
        foot(2*((i-1)*num_an+j),:) =[(r_max-(i-1)*dis_inter)/r_max*cos(anmax*pi/180),-(r_max-(i-1)*dis_inter)/r_max*sin(anmax*pi/180)];
        else
        foot(2*((i-1)*num_an+j)-1,:) =[(r_max-(i-1)*dis_inter)/r_max*cos(an_inter*(j-1)*pi/180),(r_max-(i-1)*dis_inter)/r_max*sin(an_inter*(j-1)*pi/180)];
        foot(2*((i-1)*num_an+j),:) =[(r_max-(i-1)*dis_inter)/r_max*cos(an_inter*(j-1)*pi/180),-(r_max-(i-1)*dis_inter)/r_max*sin(an_inter*(j-1)*pi/180)];
        end
    end
end
% for i =1: num_dis2            %尺蠖步态的落足点策略生成
%     for j=1:num_an2
%         if i ==num_dis2 && j ==num_an2
%         foot(num_for+2*((i-1)*num_an2+j)-1,:) =[-r_min/r_max*cos(anmax2*pi/180),r_min/r_max*sin(anmax2*pi/180)];
%         foot(num_for+2*((i-1)*num_an2+j),:) =[-r_min/r_max*cos(anmax2*pi/180),-r_min/r_max*sin(anmax2*pi/180)];
%         elseif i== num_dis2
%          foot(num_for+2*((i-1)*num_an2+j)-1,:) =[-r_min/r_max*cos(an_inter*(j-1)*pi/180),r_min/r_max*sin(an_inter*(j-1)*pi/180)];
%         foot(num_for+2*((i-1)*num_an2+j),:) =[-r_min/r_max*cos(an_inter*(j-1)*pi/180),-r_min/r_max*sin(an_inter*(j-1)*pi/180)];
%         elseif j == num_an2
%         foot(num_for+2*((i-1)*num_an2+j)-1,:) =[-(r_max-(i)*dis_inter)/r_max*cos(anmax2*pi/180),(r_max-(i)*dis_inter)/r_max*sin(anmax2*pi/180)];
%         foot(num_for+2*((i-1)*num_an2+j),:) =[-(r_max-(i)*dis_inter)/r_max*cos(anmax2*pi/180),-(r_max-(i)*dis_inter)/r_max*sin(anmax2*pi/180)];
%         else
%         foot(num_for+2*((i-1)*num_an2+j)-1,:) =[-(r_max-(i)*dis_inter)/r_max*cos(an_inter*(j-1)*pi/180),(r_max-(i)*dis_inter)/r_max*sin(an_inter*(j-1)*pi/180)];
%         foot(num_for+2*((i-1)*num_an2+j),:) =[-(r_max-(i)*dis_inter)/r_max*cos(an_inter*(j-1)*pi/180),-(r_max-(i)*dis_inter)/r_max*sin(an_inter*(j-1)*pi/180)];
%         end
%     end
% end
end
%%
%求给定步态策略下得到的落足点
function [foot_str] = footstep_strategy(apathline)
global foot;
dis_pse=norm(apathline(2,:)-apathline(1,:));       %起点到终点距离
ps = apathline(1,:);
pxy=(apathline(2,:)-apathline(1,:))/dis_pse;
TR=[pxy(1) -pxy(2);pxy(2) pxy(1)];

foot_str = dis_pse*foot;
num_str = size(foot_str,1);
for i=1:num_str
    foot_str(i,:) = (TR*foot_str(i,:)'+ps')';
end 
end
%%
function foot_hei = foot_height(land_foot)
%求落足点的高度
%[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height );
global map_bor;global acr_obs;global att_obs;global byp_obs;global hei_obs;
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
att1_obs=zeros(num_att,2);
k_byp=floor(num_byp/4);
k_acr=floor(num_acr/4);
k_att=floor(num_att/4);
att1_obs=att_obs(:,5:6);
foot_hei = 0;
for i=1:k_att
    if insidepolygon(att1_obs(i*4-3:i*4,:),land_foot)==1;
        foot_hei = hei_obs(k_byp+k_acr+i);
    end
end
end
