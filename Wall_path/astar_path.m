function [pathstr] = astar_path(reso_map,ps,pe,map_border,obs_border,obs_height)
%Example on the use of AStar Algorithm in an occupancy grid. 

[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height );               %导入地图并生成障碍区域
num_bor = size(map_bor,1);
    max_borx = max(map_bor(:,1));      
    max_bory = max(map_bor(:,2));
    min_borx = min(map_bor(:,1));
    min_bory = min(map_bor(:,2));
%     min_borxp = map_bor(1,:);
%     min_boryp = map_bor(1,:);
% for i=1:num_bor-1
%     if map_bor(i,1)<map_bor(i+1,1)
%         min_borxp = map_bor(i,:);
%     end
% end
% for i=1:num_bor-1
%     if map_bor(i,2)<map_bor(i+1,2)
%         min_boryp = map_bor(i,:);
%     end
% end
%reso_map = 0.1;  %地图网格分辨率
ranx = max_borx-min_borx;
rany = max_bory-min_bory;
num_xx = ceil(ranx/reso_map);
num_yy = ceil(rany/reso_map);

wcvec = -[min_borx,min_bory];
map_grid = zeros(num_xx,num_yy);
cost_path = zeros(num_yy,num_xx);    
  map1_bor =zeros(num_bor,2);
 num_byp=size(byp_obs,1);  
byp1_obs=zeros(num_byp,2); %绕开的障碍物区域
%byp2_obs=zeros(num_byp,2); %绕开的障碍物扩展区域
num_acr=size(acr_obs,1);
acr1_obs=zeros(num_acr,2); %跨越的障碍物区域
%acr2_obs=zeros(num_acr,2); %跨越的障碍物扩展区域
num_att=size(att_obs,1);
att1_obs=zeros(num_acr,2); %吸附的障碍物扩展区域
att2_obs=zeros(num_acr,2); %吸附的障碍物内缩区域
%att3_obs=zeros(num_acr,2); %跨越的障碍物区域

k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;

acr_stm1=zeros(k_acr,1);
att_stm1=zeros(2*k_att,1);
byp_stm1=zeros(k_byp,1);

for i=1:num_bor
    map1_bor(i,:) = map_bor(i,:)+wcvec;
end 
for i=1:num_byp                                           %障碍物及其各种区域的转化  
    byp1_obs(i,:)=byp_obs(i,3:4)+wcvec;
end
for i=1:num_acr
    acr1_obs(i,:)=acr_obs(i,3:4)+wcvec;
end
for i=1:num_att
    att1_obs(i,:)=att_obs(i,3:4)+wcvec;
    att2_obs(i,:)=att_obs(i,5:6)+wcvec;
end
ps=ps+wcvec+0.0001;
pe=pe+wcvec+0.0001;
for i =1:num_xx+1
    for j =1:num_yy+1
        map_poi = [(i-1)*reso_map, (j-1)*reso_map];
        if insidepolygon(map1_bor,map_poi) ==0
        map_grid(i,j) =1;
        continue;
        end
         for k=1:k_byp           
            byp_stm1(k) = insidepolygon(byp1_obs(k*4-3:k*4,:),map_poi);
            %byp_stm1(k_byp+j) = insidepolygon(byp2_obs(j*4-3:j*4,:),pw);
             if byp_stm1(k)==1
                 map_grid(i,j) =1;           
                continue;
             end
         end
         for k =1:k_acr
            acr_stm1(k) = insidepolygon(acr1_obs(k*4-3:k*4,:),map_poi);  
             if acr_stm1(k)==1
              %   map_grid(i,j) =1;
                 if hei_obs(k_byp+k)<=0.29
                     cost_path(j,i)=1;
                      continue;
                 else
                 cost_path(j,i)=(hei_obs(k_byp+k)-0.29)*100;
                      continue;
                 end
             end
         end
         for k =1:k_att
            att_stm1(k) = insidepolygon(att1_obs(k*4-3:k*4,:),map_poi); 
            att_stm1(k+k_att) = insidepolygon(att2_obs(k*4-3:k*4,:),map_poi); 
             if att_stm1(k)==1&&att_stm1(k+k_att)==0
              %   map_grid(i,j) =1;
                  cost_path(j,i)=hei_obs(k_byp+k_acr+k)*20;
                continue;
             end
         end
%         elseif insidepolygon(byp1_obs,map) ==1
%             map_grid(i,j) =1;
%         elseif insidepolygon(acr1_obs,point) ==1
%             map_grid(i,j) =1;
%         elseif insidepolygon(att1_obs,point) ==1
%              map_grid(i,j) =1;
        
    end
end
Psx = ceil(ps(1,1)/reso_map);
Psy = ceil(ps(1,2)/reso_map);
Pex = ceil(pe(1,1)/reso_map);
Pey = ceil(pe(1,2)/reso_map);
MAP=int8(zeros(num_yy,num_xx));
[mapx mapy] = find(map_grid==1);
for i=1:length(mapx)
    MAP(mapy(i),mapx(i))=1;
end

GoalRegister=int8(zeros(num_yy,num_xx));
GoalRegister(Pey,Pex)=1;

tic
OptimalPath=ASTARPATH(Psx,Psy,MAP,GoalRegister,cost_path,4); %A*算法求解全局路径，后面的数字越大，说明搜索的方向越多，求解时间越长

toc
num_path = size(OptimalPath,1);

for i= 1:num_path
    pathstr(num_path+1-i,:) = [(OptimalPath(i,2)-1)*reso_map,(OptimalPath(i,1)-1)*reso_map]-wcvec;
end
pathstr(1,:) = ps-wcvec-0.001;
pathstr(num_path,:) = pe-wcvec-0.001;
if size(OptimalPath,2)>1
% figure(10)
% imagesc((MAP))
% colormap(flipud(gray));
% 
% hold on
% grid on
% plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
% plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
% plot(OptimalPath(:,2),OptimalPath(:,1),'r')
% legend('Goal','Start','Path')


else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end
end   