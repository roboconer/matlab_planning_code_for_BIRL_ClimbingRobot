function [pathstr,path_dis] = path_straight( path,map_border,obs_border,obs_height )
%UNTITLED 此处显示有关此函数的摘要
%   将粒子群全局路径拉直
[ map_bor,acr_obs,att_obs,byp_obs ] = map(map_border,obs_border,obs_height); 
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
byp1_obs=zeros(num_byp,2);  %绕开的障碍区域
att1_obs=zeros(num_att,2);
k_byp=floor(num_byp/4);
k_acr=floor(num_acr/4);
k_att=floor(num_att/4);
acr_stm1=zeros(k_acr,1);
acr_stm2=zeros(k_acr,1);
att_stm1=zeros(k_att,1);
att_stm12=zeros(k_att,1);
att_stm2=zeros(k_att,1);
byp_stm1=zeros(k_byp,1);
byp_stm2=zeros(k_byp,1);
byp1_obs=byp_obs(:,3:4);
att1_obs=att_obs(:,1:2);
att2_obs=att_obs(:,3:4);
num_pa=size(path,1);
s=1;
pathstr(1,:)=path(1,:);
aj=2;
k=2;
  
while s<num_pa
  
    for i=s:num_pa
        %ak=0;
        if i+k>num_pa
          s=i+k-1;
          break;
        end
       l1=[path(s,:);path(i+k,:)];
       %检测路径点是否在各种障碍区域内部或者与各种障碍区域是否相交
        for j=1:k_byp
        byp_stm1(j) = insidepolygon(byp1_obs(j*4-3:j*4,:),l1(2,:));
         if byp_stm1(j)==1

                 break;
         end
       [inter_l1,byp_stm2(j)] =linepolyintersection(l1,byp1_obs(j*4-3:j*4,:),2);
          if byp_stm2(j)>=1

                 break;
          end
        end
  
       for j=1:k_acr
                 acr_stm1(j) = insidepolygon(acr_obs(j*4-3:j*4,3:4),l1(2,:));
                 if acr_stm1(j)==1
                    
                     break;
                 end
           [inter_l1,acr_stm2(j)] =linepolyintersection(l1,acr_obs(j*4-3:j*4,3:4),2);
            if acr_stm2(j)==2
                   
                     break;
            end
       end
       for j=1:k_att             
                 att_stm1(j) = insidepolygon(att1_obs(j*4-3:j*4,:),l1(2,:));
                 if att_stm1(j)==1
                     break;
                 end
         
         end
  
        for j=1:k_att             
                 att_stm12(j) = insidepolygon(att2_obs(j*4-3:j*4,:),l1(2,:));
                 if att_stm1(j)==1
                     break;
                 end
           [inter_l1,att_stm2(j)] = linepolyintersection(l1,att2_obs(j*4-3:j*4,:),2);
               if att_stm2(j)==2
                     break;
               end
         end

       if all(byp_stm1 < 1) && all(byp_stm2 < 1) && all(acr_stm1 < 1) && all(acr_stm2 < 2) && all(att_stm12 < 1) && all(att_stm2 < 2) || any(att_stm1 == 1)
           continue;
       else
          % if i+k-s>1
           pathstr(aj,:)=path(i+k-1,:);
           s=i+k-1; 
%            else
%           pathstr(aj,:)=path(i+k,:);
%            s=i+k; 
%            end
            aj=aj+1;
           break;
       end
    end
end
pathstr(aj,:)=path(num_pa,:);
path_dis = 0;
for i=1:size(pathstr,1)-1
    path_dis = path_dis+norm(pathstr(i+1,:)-pathstr(i,:));
end
% plot(pathstr(:,1)*1000,pathstr(:,2)*1000,'-r','LineWidth',2);
% plot(pathstr(1,1)*1000,pathstr(1,2)*1000,'or','LineWidth',2);
% plot(pathstr(aj,1)*1000,pathstr(aj,2)*1000,'or','LineWidth',2);
end

