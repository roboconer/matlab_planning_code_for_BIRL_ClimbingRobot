function [ se_footstep3d ] = footstep_sequence( pathstr,map_border,obs_border,obs_height )

%   根据规划好的全局路径与爬壁机器人步距范围输出离散的落足点，利用有限种步态策略搜索与评估方法
global foot;global map_bor;global acr_obs;global att_obs;global byp_obs;global hei_obs;
[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height ); 
tic
% foot= footgen(0.05,10,60);   %根据参数生成落足点策略，第一个参数为距离间隔，第二个为角度间隔，第三个为角度范围（包括顺逆时针）
foot= footgen(0.05,10,60); 
r_max=0.58;
r_min=0.4;

num_pa=size(pathstr,1);
ciro = pathstr(1,:);
se_footstep(1,:)=ciro;
num_step =1;
num_fea =1;
com_len=10000; 
num_r=1;
[se_footstep,foot_hei,num_step,state,last2_foot,last2_num,last2_tar,last2_len,last2_str,last_step] = footstepsearch(pathstr(1,:),pathstr);
plot(se_footstep(:,1)*1000,se_footstep(:,2)*1000,'-*m','LineWidth',2);
if num_step==1 && state ==0
     h=msgbox('Sorry, No path exists to the Target for the first step!','warn');
     uiwait(h,3);
      return;
elseif state ==2
    ciro = se_footstep(num_step,:);
    [se_footstep2,foot_hei2,num_step2,state,~,~,~,~,~,~] = footstepsearch(ciro,pathstr);
    if state ==1
        se_footstep(num_step:num_step+num_step2,:)=se_footstep2(1:num_step2+1,:);
        foot_hei(num_step:num_step+num_step2) = foot_hei2(1:num_step2+1);       
    end
elseif state == 0
     for j = 1: last2_num-1
     com2_len = 10000;
         for i = 1:last2_num
         if norm(last2_str(last2_foot(i),:)-last2_tar)<com2_len && norm(last2_str(last2_foot(i),:)-last2_tar)>last2_len
             com2_len = norm(last2_str(last2_foot(i),:)-last2_tar);
             ciro = last2_str(last2_foot(i),:);
            %se_footstep(num_step,:) = last2_str(last2_foot(i),:);
         end         
         end 
         [se_footstep2,foot_hei2,num_step2,state,~,~,~,~,~,~] = footstepsearch(ciro,pathstr);
         if state>=1
            se_footstep(num_step-1:num_step+num_step2-1,:)=se_footstep2(1:num_step2+1,:);
            foot_hei(num_step-1:num_step+num_step2-1) = foot_hei2(1:num_step2+1);
             break;
         end
         last2_len = com2_len;
        num_r = num_r+1;
    end
end
toc
% if num_r==last2_num||state ==0
% if state == 0 &&norm(last_step)>0
%     ciro = last_step;
%     [se_footstep3,foot_hei3,num_step3,state,~,~,~,~,~,~] = footstepsearch(ciro,pathstr);
%     if state>0
%          se_footstep(num_step-1:num_step+num_step3-1,:)=se_footstep3(1:num_step3+1,:);
%             foot_hei(num_step-1:num_step+num_step3-1) = foot_hei3(1:num_step3+1);
%     end 
% end
if state == 0
     h=msgbox('Sorry, No path exists to the Target for the middle or last several steps!','warn');
     uiwait(h,3);
      return;
end
%foot_hei(1)=0;
% while norm(ciro-pathstr(num_pa,:))>r_max
% [lici,tar] = pathcircleinter(pathstr,ciro,r_max);
% apathline = [ciro;lici];
% foot_hei(num_step) = foot_height(ciro); 
% %fohe = foot_height(lici);
% %foot_hei(num_step) = foot_height(lici); 
% [foot_str] = footstep_strategy(apathline);
% num_str = size(foot_str,1);
% %state_judge(num_step)  = footstep_amendment(apathline,[foot_hei(num_step),fohe]);
%     for i=1:num_str
%         apathline =[ciro;foot_str(i,:)];
%         fohe = foot_height(foot_str(i,:)); 
%         state_judge(num_step)  = footstep_amendment(apathline,[foot_hei(num_step),fohe]);
%         if  state_judge(num_step) == 1
%             feafoot(num_fea) =i;
%             num_fea=num_fea+1;
% 
%         else
%             continue;
%         end
%     end
%      if num_fea ==1   %当前步附着点找不到有效解，回溯到上一步
%          com2_len = 10000;
%          for i = 1:last_num
%          if norm(last_str(last_foot(i),:)-last_tar)<com2_len&&norm(last_str(last_foot(i),:)-last_tar)>last_len
%              com2_len = norm(last_str(last_foot(i),:)-last_tar);
%              ciro = last_str(last_foot(i),:);
%             se_footstep(num_step,:) = last_str(last_foot(i),:);
%          end         
%          end   
%          last_len = com2_len;
%         num_r = num_r+1;
%         if num_r<=last_num
%          continue;
%         end
%             h=msgbox('Sorry, No path exists to the Target!','warn');
%              uiwait(h,3);
%              return;
%      end
%      for i = 1:num_fea-1
%          if norm(foot_str(feafoot(i),:)-tar)<com_len
%              com_len = norm(foot_str(feafoot(i),:)-tar);
%             ciro = foot_str(feafoot(i),:);
%             se_footstep(1+num_step,:) = foot_str(feafoot(i),:);
%         end 
%      end   
%        
% %end
% for i=1:num_fea-1
% last_foot(i) = feafoot(i);
% end 
% last_num = num_fea-1;
% last_tar = tar;
% last_len = com_len;
% num_r =1;
% for i=1:size(foot_str,1)
%     last_str(i,:) = foot_str(i,:);
% end
% num_fea =1;
% feafoot=0;
% com_len = 10000;
% num_step = num_step+1;
% plot(foot_str(:,1)*1000,foot_str(:,2)*1000,'+g','LineWidth',2);
% foot_str =0;
% end

% foot_hei(num_step) = foot_height(ciro);
% foot_hei(num_step) = foot_height(se_footstep(num_step,:));

% if norm(se_footstep(num_step,:)-pathstr(num_pa,:))>0.4
% foot_hei(num_step+1) = 0;
% se_footstep(1+num_step,:)=pathstr(num_pa,:);
% else
%     for j = 1: last_num-1
%      com2_len = 10000;
%          for i = 1:last_num
%          if norm(last_str(last_foot(i),:)-last_tar)<com2_len && norm(last_str(last_foot(i),:)-last_tar)>last_len
%              com2_len = norm(last_str(last_foot(i),:)-last_tar);
%              ciro = last_str(last_foot(i),:);
%             se_footstep(num_step,:) = last_str(last_foot(i),:);
%          end         
%          end  
%          if norm(se_footstep(num_step,:)-pathstr(num_pa,:))>0.4
%              foot_hei(num_step) = foot_height(ciro);
%              foot_hei(num_step+1) = 0;
%              se_footstep(1+num_step,:)=pathstr(num_pa,:);
%              break;
%          end
%          last_len = com2_len;
%         num_r = num_r+1;
%     end
%     if num_r==last_num
%         h=msgbox('Sorry, No path exists to the Target!','warn');
%              uiwait(h,3);
%              return;
%     end 
% end
plot(se_footstep(:,1)*1000,se_footstep(:,2)*1000,'-*k','LineWidth',2);
% for i = 1:size(se_footstep(:,1))-1
%     quiver(se_footstep(i,1)*1000,se_footstep(i,2)*1000,se_footstep(i+1,1)*1000-se_footstep(i,1)*1000,se_footstep(i+1,2)*1000-se_footstep(i,2)*1000);
% end
for i=1:length(foot_hei)
    se_footstep3d(i,:) = [se_footstep(i,:),foot_hei(i)];
end

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
% st=[dis_pse,0]; 
% stl10=[dis_pse*cos(10*pi/180),dis_pse*sin(10*pi/180)];
% str10=[dis_pse*cos(10*pi/180),-dis_pse*sin(10*pi/180)];
% stl20=[dis_pse*cos(20*pi/180),dis_pse*sin(20*pi/180)];
% str20=[dis_pse*cos(20*pi/180),-dis_pse*sin(20*pi/180)];
% st5 = [dis_pse/58*53,0];
% stl30=[dis_pse*cos(30*pi/180),dis_pse*sin(30*pi/180)];
% str30=[dis_pse*cos(30*pi/180),-dis_pse*sin(30*pi/180)];
% st10 = [dis_pse/58*48,0];
% stl40=[dis_pse*cos(40*pi/180),dis_pse*sin(40*pi/180)];
% str40=[dis_pse*cos(40*pi/180),-dis_pse*sin(40*pi/180)];
% st15 = [dis_pse/58*43,0];
% stl50=[dis_pse*cos(50*pi/180),dis_pse*sin(50*pi/180)];
% str50=[dis_pse*cos(50*pi/180),-dis_pse*sin(50*pi/180)];
% st18 = [dis_pse/58*40,0];
% foot_str=[st;stl10;str10;stl20;str20;st5;stl30;str30;st10;stl40;str40;st15;stl50;str50;st18];
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
%%
function [se_footstep,foot_hei,num_step,state,last2_foot,last2_num,last2_tar,last2_len,last2_str,last_st] = footstepsearch(startp,pathstr)
r_max=0.58;
%r_min=0.2; % 2020.2.20 改了这个参数，原来是0.399
r_min=0.399;
num_pa=size(pathstr,1);
ciro = startp;
se_footstep(1,:)=ciro;
num_step =1;
num_fea =1;
com_len=10000; 
state = 0;
last2_foot=0;last2_num=0;last_num = 0;last2_tar=0;last2_len=0;last2_str=zeros(100,2);last_st =zeros(1,2);
if norm(ciro-pathstr(num_pa,:))<r_min              %起始点和目标点之间小于一步可到达的最小距离
    
     %生成步态策略
     vec = (pathstr(num_pa,:)-ciro)/norm(pathstr(num_pa,:)-ciro)*r_max;
     apathline = [ciro;vec+ciro];
     apathline2 = [ciro;-vec+ciro];
     
     %2020.2.21添加：步态策略添加水平垂直方向
     d1 = 60;
     d2 = -d1;
     rotationMatrix1 = [0, -1; 1, 0]; %90°
     rotationMatrix2 = [0, 1; -1, 0]; %-90°
%      rotationMatrix3 = [cosd(d1), -sind(d1); sind(d1), cosd(d1)]; 
%      rotationMatrix4 = [cosd(d2), -sind(d2); sind(d2), cosd(d2)]; 
     apathline3 = [ciro; (rotationMatrix1 * vec')'+ciro];
     apathline4 = [ciro; (rotationMatrix2 * vec')'+ciro];
%      apathline5 = [ciro; (rotationMatrix3 * vec')'+ciro];
%      apathline6 = [ciro; (rotationMatrix4 * vec')'+ciro];
     
     [foot_str] = footstep_strategy(apathline);
     [foot_str2] = footstep_strategy(apathline2);
     [foot_str3] = footstep_strategy(apathline3);
     [foot_str4] = footstep_strategy(apathline4);
%      [foot_str5] = footstep_strategy(apathline5);
%      [foot_str6] = footstep_strategy(apathline6);
     
     %前后方向整合到一起
     num_f1 = size(foot_str,1);
     for i=1:size(foot_str2,1)
         foot_str(num_f1+i,:) = foot_str2(i,:);
     end
     num_f1 = size(foot_str,1);
     for i=1:size(foot_str3,1)
         foot_str(num_f1+i,:) = foot_str3(i,:);
     end
     num_f1 = size(foot_str,1);
     for i=1:size(foot_str4,1)
         foot_str(num_f1+i,:) = foot_str4(i,:);
     end
%      num_f1 = size(foot_str,1);
%      for i=1:size(foot_str5,1)
%          foot_str(num_f1+i,:) = foot_str5(i,:);
%      end
%      num_f1 = size(foot_str,1);
%      for i=1:size(foot_str6,1)
%          foot_str(num_f1+i,:) = foot_str6(i,:);
%      end
     
     %num_str = size(foot_str,1)+size(foot_str2,1);%%%%%%%%%%
     num_str = size(foot_str,1);
     foot_hei(num_step) = foot_height(ciro);
     %判断生成的落足点符不符合要求
     for i=1:num_str
          apathline =[ciro;foot_str(i,:)];
          dis2 = norm(pathstr(num_pa,:)-foot_str(i,:));
          state_judge(num_step)  = footstep_amendment(apathline,[foot_height(ciro),foot_height(foot_str(i,:))]);
          apathline2 = [foot_str(i,:);pathstr(num_pa,:)];
          state_judge(num_step+1) = footstep_amendment(apathline2,[foot_height(foot_str(i,:)),foot_height(pathstr(num_pa,:))]);
          if dis2>=r_min&&dis2<=r_max&&state_judge(num_step)==1&&state_judge(num_step+1)==1
              num_step= num_step+1;
              foot_hei(num_step) = foot_height(foot_str(i,:));
              se_footstep(num_step,:) = foot_str(i,:);
              foot_hei(num_step+1) = foot_height(pathstr(num_pa,:));
              se_footstep(num_step+1,:) = pathstr(num_pa,:);      
              state = 1;
              return;
          end
     end
     if state ==0
     h=msgbox('Sorry, The distance from starting to the Target is too short!','warn'); %起始点和目标点之间的距离过小且当前策略落足点找不到有效解
     uiwait(h,3);
     return;
     end;
end
while norm(ciro-pathstr(num_pa,:))>r_max       %起始点和目标点之间大于一步可到达的最大距离
[lici,tar] = pathcircleinter(pathstr,ciro,r_max);
apathline = [ciro;lici];
foot_hei(num_step) = foot_height(ciro); 
[foot_str] = footstep_strategy(apathline);
num_str = size(foot_str,1);
    for i=1:num_str
        apathline =[ciro;foot_str(i,:)];
        fohe = foot_height(foot_str(i,:)); 
        state_judge(num_step)  = footstep_amendment(apathline,[foot_hei(num_step),fohe]);
        if  state_judge(num_step) == 1
            feafoot(num_fea) =i;
            num_fea=num_fea+1;            
        else
            continue;
        end
    end
     if num_fea ==1 && norm(ciro-startp)<0.0001  %第一步就找不到有效解，策略搜索不可行
         state = 0;
         return;
     elseif num_fea ==1          %当前步附着点找不到有效解，回溯到上一步
         com2_len = 10000;
         for i = 1:last_num
         if norm(last_str(last_foot(i),:)-last_tar)<com2_len&&norm(last_str(last_foot(i),:)-last_tar)>last_len
             com2_len = norm(last_str(last_foot(i),:)-last_tar);
             ciro = last_str(last_foot(i),:);
            se_footstep(num_step,:) = last_str(last_foot(i),:);
         end         
         end   
         last_len = com2_len;
        num_r = num_r+1;
        if num_r<=last_num
         continue;
        end
           state = 0;
         return;
     end
     for i = 1:num_fea-1                %在可行的附着点里面选择最优的点：离引导路径最近且能走的越远
         if norm(foot_str(feafoot(i),:)-tar)<com_len
             com_len = norm(foot_str(feafoot(i),:)-tar);
            ciro = foot_str(feafoot(i),:);
            se_footstep(1+num_step,:) = foot_str(feafoot(i),:);
        end 
     end   
       
if num_step >2    %记录上两步的数据
    for i =1:last_num
        last2_foot(i)=last_foot(i);
    end
    last2_num = last_num;
    last2_tar = last_tar;
    last2_len = last_len;
  for i = 1: size(last_str,1)
      last2_str(i,:) = last_str(i,:);
  end
end 
for i=1:num_fea-1         %记录上一步的数据
last_foot(i) = feafoot(i);
end 
last_num = num_fea-1;
last_tar = tar;
last_len = com_len;
last_step = se_footstep(1+num_step,:);
num_r =1;
for i=1:size(foot_str,1)
    last_str(i,:) = foot_str(i,:);
end
num_fea =1;
feafoot=0;
com_len = 10000;
num_step = num_step+1;

plot(foot_str(:,1)*1000,foot_str(:,2)*1000,'+g','LineWidth',1);

foot_str =0;
end

num_r=1;
apathline = [ciro;pathstr(num_pa,:)];
foot_hei(num_step) = foot_height(ciro);
state_judge(num_step)  = footstep_amendment(apathline,[foot_hei(num_step),0]);
%当一步即可到达目标点的情况处理或者剩最后一步即可到达目标点的情况处理
 if norm(se_footstep(num_step,:)-pathstr(num_pa,:))>=r_min && state_judge(num_step)==1   %起始点和目标点在一步运动范围内且能直接一步到达
   foot_hei(num_step+1) = foot_height(pathstr(num_pa,:));
   se_footstep(1+num_step,:)=pathstr(num_pa,:);
   state = 1;
    return;
 elseif last_num ==0 && norm(se_footstep(num_step,:)-pathstr(num_pa,:))>=r_min    %起始点和目标点在一步运动范围内但不能直接一步到达的情况，看情况判断是否需要添加步数
     vec = (pathstr(num_pa,:)-ciro)/norm(pathstr(num_pa,:)-ciro)*r_max;
     apathline = [ciro;vec+ciro];
     [foot_str] = footstep_strategy(apathline);
    
     plot(foot_str(:,1)*1000,foot_str(:,2)*1000,'+g','LineWidth',1);

     num_str = size(foot_str,1);
     foot_hei(num_step) = foot_height(ciro);
     for i= 1:num_str
       apathline =[ciro;foot_str(i,:)];
       apathline2 =[foot_str(i,:);pathstr(num_pa,:)];
       dis2 = norm(pathstr(num_pa,:)-foot_str(i,:));
       state_judge(num_step) = footstep_amendment(apathline,[foot_height(ciro),foot_height(foot_str(i,:))]);
       state_judge(num_step+1) = footstep_amendment(apathline2,[foot_height(foot_str(i,:)),foot_height(pathstr(num_pa,:))]);
       if state_judge(num_step) ==1 && state_judge(num_step+1) ==1 && dis2>=r_min && dis2<=r_max
           foot_hei(num_step+1) = foot_height(foot_str(i,:));
           se_footstep(1+num_step,:)=foot_str(i,:);
           num_step = num_step+1;
           state =2;
           return;
       end
       if state_judge(num_step) ==1 && dis2<r_min
           foot_hei(num_step+1) = foot_height(foot_str(i,:));
           se_footstep(1+num_step,:)=foot_str(i,:);
           num_step = num_step+1;
           state =2;
           return;
       end
     end
     if state ==0
         h=msgbox('Sorry, No path exists to the Target for the first step!','warn');
         uiwait(h,3);
     end 
 elseif last_num>0
    for j = 1: last_num-1
     com2_len = 10000;
         for i = 1:last_num
         if norm(last_str(last_foot(i),:)-last_tar)<com2_len && norm(last_str(last_foot(i),:)-last_tar)>last_len
             com2_len = norm(last_str(last_foot(i),:)-last_tar);
             ciro = last_str(last_foot(i),:);
            se_footstep(num_step,:) = last_str(last_foot(i),:);
         end
         end 
         apathline = [ciro;pathstr(num_pa,:)];
         foot_hei(num_step) = foot_height(ciro);
         state_judge(num_step) = footstep_amendment(apathline,[foot_hei(num_step),foot_height(pathstr(num_pa,:))]);
         if norm(se_footstep(num_step,:)-pathstr(num_pa,:))>=r_min && norm(se_footstep(num_step,:)-pathstr(num_pa,:))<=r_max && state_judge(num_step)==1
             foot_hei(num_step) = foot_height(ciro);
             foot_hei(num_step+1) = foot_height(pathstr(num_pa,:));
             se_footstep(1+num_step,:)=pathstr(num_pa,:);
             state = 1;
             return;
         end
         last_len = com2_len;
        num_r = num_r+1;
    end
    if num_r==last_num
           last_st = last_step;           %添加还差最后一步到达目标点的最优候选点，
           state = 0;
            return;
    end 
 end
end