function [ dis,po1,po2] = dis_compute(testobj,obsp,option)
%  机器人与障碍物之间求距与距离最近点,testobj为测距对象，obsp为测距障碍物  ,po1为测试的点，po2为障碍物上的点
%option为选择何种元素测距，包括点与障碍物求距（1），线段与障碍物求距（2），点与壁面求距(3)，线段与壁面求距(4),多边形与多边形求距（5）,多边形与障碍物求距（6）
%poly =[0 0 2;5 0 2;5 4 2];
% poly2=[-0.7 -0.3 0.1;-0.7 -0.4 0.1;-0.5 -0.6,1];
% point = [6 3 3];
% lineseg=[8 2 0;8 2 7];
%obsp = [-0.6 -0.5 0;-0.5 -0.5 0;-0.5 -0.2 0;-0.6 -0.2 0;-0.6 -0.5 0.2;-0.5 -0.5 0.2;-0.5 -0.2 0.2;-0.6 -0.2 0.2];
dis = 10000;
po1 = zeros(1,3);
po2 = zeros(1,3);
if option == 1
    [obs_sur] =obp_tran(obsp);
    for i=1:5
        num_p = size(obs_sur,2)/3;
        for j=1:num_p
        poly(j,:) = obs_sur(i,j*3-2:j*3,1);
        end
        [dist,p1,p2] = pointtopoly(testobj,poly);
        if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
    end
end
% for i=1:5                                 %画出结果
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%          plot3(sur_x, sur_y,sur_z,'-r','LineWidth',2);
%          hold on;
% plot3(testobj(1),testobj(2),testobj(3),'*b','LineWidth',3);
%  plot3(po1(1,1),po1(1,2),po1(1,3),'*b','LineWidth',3);
%  plot3(po2(1,1),po2(1,2),po2(1,3),'*k','LineWidth',3);
%mArrow3([po2(1,1),po2(1,2),po2(1,3)],[po1(1,1),po1(1,2),po1(1,3)],'color','m','stemWidth',25,'facealpha',0.7);
%end
    if option == 2
         [obs_sur] =obp_tran(obsp);
      for i=1:5
        num_p = size(obs_sur,2)/3;
        for j=1:num_p
        poly(j,:) = obs_sur(i,j*3-2:j*3,1);
        end
        [dist,p1,p2] = linesegtopoly(testobj,poly);
         if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
      end
    end
%      for i=1:5
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%          plot3(sur_x, sur_y,sur_z,'-r','LineWidth',2);
%          hold on;
% plot3(testobj(:,1),testobj(:,2),testobj(:,3),'-b','LineWidth',3)
% plot3([po1(1,1),po2(1,1)],[po1(1,2),po2(1,2)],[po1(1,3),po2(1,3)],'-b','LineWidth',3)
 %    end
    if option == 3
        [dist,p1,p2] = pointtopoly(testobj,obsp);
        if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
    end
    if option == 4
       [dist,p1,p2] = linesegtopoly(testobj,obsp);
       if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
       end
    end
%       plot3(testobj(:,1),testobj(:,2),testobj(:,3),'-b','LineWidth',3);
%     hold on;
%     plot3(obsp(:,1),obsp(:,2),obsp(:,3),'-r','LineWidth',3);
%      plot3([po1(1,1),po2(1,1)],[po1(1,2),po2(1,2)],[po1(1,3),po2(1,3)],'-b','LineWidth',3)
if option == 5
    num_bor = size(testobj,1);
    for i = 1:num_bor
        if i == num_bor
            bor = [testobj(num_bor,:);testobj(1,:)];
        else
            bor = [testobj(i,:);testobj(i+1,:)];
        end
        [dist,p1,p2] = linesegtopoly(bor,obsp);
        if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
       end
    end
end
if option == 6
    num_bor  = size(testobj,1);
     [obs_sur] =obp_tran(obsp);
    for i=1:5
        num_p = size(obs_sur,2)/3;
        for j=1:num_p
        poly(j,:) = obs_sur(i,j*3-2:j*3,1);
        end
        [dist,p1,p2] =dis_compute(testobj,poly,5);
        if dist < dis
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
    end   
    
%  fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
%  patch('Faces',fac_obs,'Vertices',obsp,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',0.3);
%          hold on;
% fac = [1 2 3];
%  patch('Faces',fac,'Vertices',testobj,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',0.3);
%   plot3([po1(1,1),po2(1,1)],[po1(1,2),po2(1,2)],[po1(1,3),po2(1,3)],'-b','LineWidth',3);
end
end

function [dis,po1,po2] = linesegtopoly(lineseg,poly)%线段与面求距,返回最短距离与相应的两点,po1为线段上的点，po2为面上的点
num_p = size(poly,1);
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
lin = lineseg(2,:)-lineseg(1,:);            %先求解线段是否与面相交，相交距离为0；      
kk = dot(vec,poly(1,:)-lineseg(1,:))/dot(vec,lineseg(2,:)-lineseg(1,:)); 
interp = lineseg(1,:) + lin*kk;
state1 = insidepoly3(interp,poly);
if kk >= 0&&kk <= 1&&state1 == 1
    dis = 0;
    po1 = interp;
    po2 = interp;
    return;
end
[state,pro1] = point_pro_poly(lineseg(1,:),poly);    %求线段端点到面的最近距离
if state ==1
lin1 = norm(lineseg(1,:)-pro1);
else 
    lin1 = 10000;
end
[state,pro2] = point_pro_poly(lineseg(2,:),poly);
if state ==1
lin2 = norm(lineseg(2,:)-pro2);
else 
    lin2 = 10000;
end
%dis = min([lin1,lin2]);
if lin1<lin2
    dis = lin1;
    po1 = lineseg(1,:);
    po2 = pro1;
else
    dis = lin2;
    po1 = lineseg(2,:);
    po2 = pro2;
end
for i=1:num_p                    %求线段与线段间的最近距离
    if i == num_p
        seg = [poly(num_p,:);poly(1,:)];
       [dist,sp1,sp2] = Seg2SegDist(lineseg,seg);
    else
        seg = [poly(i,:);poly(i+1,:)];
        [dist,sp1,sp2] = Seg2SegDist(lineseg,seg);
    end
    if dist < dis
        dis = dist;
        po1 = sp1;
        po2 = sp2;
    end
end
end

function [dis,po1,po2] = pointtopoly(poi,poly)       %点与面求距,返回最短距离与相应的两点,po1原来的点，po2为面上的点
 state1 =  insidepoly3(poi,poly);
 num_p = size(poly,1);
 if state1 == 1
     dis = 0;
     po1 = poi;
     po2 = poi;
     return;
 end
 [state,pro] = point_pro_poly(poi,poly);  %先求点在面上的投影点及距离
 if state ==1
     dis = norm (poi-pro);
     po1 = poi;
     po2 = pro;
 else
     dis = 10000;
     po1 = poi;
     po2 = pro;
 end
 for i= 1:num_p                   %求点与面上边的最小距离
     if i == num_p
         line = [poly(num_p,:);poly(1,:)];
     else
         line = [poly(i,:);poly(i+1,:)];
     end
     [dist,lp] = point_to_line(poi,line);
     if dist < dis
         dis = dist;
         po1 = poi;
         po2 = lp;
     end
 end
end

function [dis,lp] = point_to_line(poi,line)   %求点到线段的最小距离，返回距离及对应的点
   if norm(line(1,:) - line(2,:))<1e-6
       dis = norm(line(1,:) - poi);
       lp = line(1,:);
       return;
   end
   vk = line(2,:) - line(1,:);
   v1 = poi - line(1,:);
   v2 = poi - line(2,:);
   if dot(vk,v1) < 1e-6
       lp = line(1,:);
       dis = norm(v1);
       return;
   end
   if dot(vk,v2) > 1e-6
       lp = line(2,:);
       dis = norm(v2);
       return;
   end
       dis = norm(cross(vk,v1)/norm(vk));
       lp = line(1,:) + vk*(dot(vk,v1)/dot(vk,vk));  
end
 
function [state,pro] = point_pro_poly(point,poly)      %点在多边形内部的投影
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
vec = vec/norm(vec);
prop = point-vec*dot(point-poly(1,:),vec);
if insidepoly3(prop,poly)==1
    state = 1;
    pro = prop;
else
    state = 0;
    pro = 0;
end
end

function [state] = insidepoly3(point,poly)        %判断点是否在空间多边形内部
num_p=size(poly,1);
mean_p=zeros(1,3);
kk=0;
for i=1:num_p
mean_p = mean_p + poly(i,:);
end
mean_p = mean_p/num_p;
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
if abs(dot(point-poly(1,:),vec))>1e-5
    state = 0;
    return;
end
for i=1:num_p
    ls = poly(i,:);
    if i==num_p
    le = poly(1,:);
    else
        le = poly(i+1,:);
    end
    if (dot(cross(point-ls,le-ls),cross(mean_p-ls,le-ls))<0)
         kk=1;
        break;
    end
end
if kk==0
    state =1;
else
    state=0;
end
end

function[obs_sur] =obp_tran(obp)%将代表障碍物的8个三维空间点转化为障碍物元素（5个面元素和8个线段元素）
    num_ob = size(obp,1)/8;
    obs_sur = zeros(5,4*3,num_ob);
    for i = 1:num_ob
        for j=1:4
            if j==4
               obs_sur(j,:,i)=[obp(8*i-4,:),obp(8*i-7,:),obp(8*i-3,:),obp(8*i,:)]; 
            else
                obs_sur(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:),obp(8*i-3+j,:),obp(8*i-4+j,:)];
            end  
        end
        obs_sur(5,:,i)=[obp(8*i-3,:),obp(8*i-2,:),obp(8*i-1,:),obp(8*i,:)];
    end
end
















