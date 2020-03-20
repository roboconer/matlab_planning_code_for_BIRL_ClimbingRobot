function [ P_path,path_l] = DataManage( particle,velocity,pbest,gbest,max_iteration,swarm_size,particle_size,n_size,fitness_size,C )
%DATAMANAGE Summary of this function goes here
%   Detailed explanation goes here
 [A,num_point,num_sur,Pstart,Pend] = surfaces;
temp=zeros(1,fitness_size);
N = zeros(1,fitness_size-1);
P = zeros(fitness_size-1,3);
Pup = zeros(fitness_size-1,3);
P2 = zeros(fitness_size-1,3);
cir_n = zeros(num_sur-1,3);
P_path = zeros(2*(num_sur-1),3);
path_l= 0;
L1 = 0.335;
L2 = 0.293;
for k=1:max_iteration
    temp(1,:)=gbest(1,:,k);
    save('ResultHistoryGbest','temp','-ascii','-tabs','-append');
end
temp(1,:)=gbest(1,:,max_iteration);
save('ResultFinalGbest','temp','-ascii','-tabs','-append');

for i=1:fitness_size-1
    N(i) = ceil(gbest(1,i,max_iteration)*n_size(1,i));
end
    for i = 1:fitness_size-1
        P(i,:) = C{i}(N(i),:);
    end
for i=1:num_sur
    Vz(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量,建立空间坐标系
    Vz_norm=norm(Vz(i,:));
    Vz(i,:)=Vz(i,:)/Vz_norm;
end
for i = 1:num_sur
for j = 1:num_point(i)
    Aup{i}(j,:) = A{i}(j,:)+ Vz(i,:)*L1;
end
end
for i = 1:num_sur - 1
    cir_n(i,:) = cross(Vz(i,:),Vz(i+1,:));
end
for i = 1:num_sur-1
    Pup(i,:) = P(i,:) + Vz(i,:)*L1;
    wospa = cir_seem_poly( Pup(i,:),cir_n(i,:),2*L2,100 );%工作空间圆
    [max_dis,po] = linetopoly(wospa,Aup{i+1},Pup(i,:));
    if norm(po)<1e-5
        [max_dis,po] = linetopoly(Aup{i+1},wospa,Pup(i,:));
        if norm(po)<1e-5
          h=msgbox('找不到相交点','warn'); 
          uiwait(h,2);
          return;
        end
    end
    P2(i,:) = po-Vz(i+1,:)*L1;
    
end
for i =1: num_sur - 1
    P_path(2*i-1,:) = P(i,:);
    P_path(2*i,:) = P2(i,:);
end
P_path =[Pstart;P_path;Pend];
 
for i = 1:size(P_path,1)-1
    path_l = path_l + norm(P_path(i,:)-P_path(i+1,:));
end
plot3(P_path(:,1)*1000,P_path(:,2)*1000,P_path(:,3)*1000,'-k*','LineWidth',1);
hold on;
plot_surface3;
end

function [max_dis,po] = linetopoly(testobj,poly,o)%求多边形与多边形相交的点，且离某定点最远,o为定点
max_dis = 0;
po = zeros(1,3);
 num_bor = size(testobj,1);
    for i = 1:num_bor
        if i == num_bor
            bor = [testobj(num_bor,:);testobj(1,:)];
        else
            bor = [testobj(i,:);testobj(i+1,:)];
        end
        [dist,p1,p2] = linesegtopoly(bor,poly);
        if dist <1e-4 
            if norm(p1-o)>max_dis
                max_dis = norm(p1-o);
                po = p1;
            end
        end   
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