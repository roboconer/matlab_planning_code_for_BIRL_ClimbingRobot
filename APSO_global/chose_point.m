function [ C ] = chose_point(  )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
[A1,num_point,num_sur,Pstart,Pend] = surfaces;
r=0.2;   %吸盘半径
Vz=zeros(num_sur,3);
Vy=zeros(num_sur,3);
Vx=zeros(num_sur,3);
cir_n = zeros(num_sur-1,3);
pw = zeros(num_sur,3);
psw= zeros(num_sur,3);
L1 = 0.335;
L2 = 0.293;
% Pstart = zeros(1,3);
% Pend = zeros(1,3);
R_sw = zeros(3,3,num_sur);%平面坐标系在世界坐标系下的转换矩阵
R_ws = zeros(3,3,num_sur);%世界坐标系在平面坐标系下的转换矩阵
Xa=[1,0,0];
Ya=[0,1,0];
Za=[0,0,1];
reso_map = 0.05;  %地图网格分辨率
for i=1:num_sur
    v1= (A1{i}(num_point(i),:)-A1{i}(1,:))/norm(A1{i}(num_point(i),:)-A1{i}(1,:));
    v2= (A1{i}(2,:)-A1{i}(1,:))/norm(A1{i}(2,:)-A1{i}(1,:));
    ang_cos = dot(v1,v2);
    ang_sin = sqrt(1-ang_cos^2);
    l = r/ang_sin;
    A{i}(1,:) = A1{i}(1,:)+ v1*l +v2*l;
    
    v1=( A1{i}(1,:)-A1{i}(num_point(i),:))/norm(A1{i}(1,:)-A1{i}(num_point(i),:));
    v2= (A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:))/norm(A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:));
    ang_cos = dot(v1,v2);
    ang_sin = sqrt(1-ang_cos^2);
    l = r/ang_sin;
    A{i}(num_point(i),:) = A1{i}(num_point(i),:)+ v1*l +v2*l;
    
   for j=2:num_point(i)-1
       v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
       v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
       ang_cos = dot(v1,v2);
       ang_sin = sqrt(1-ang_cos^2);
       l = r/ang_sin;
       A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;   
   end
end
for i= 1:num_sur
    B{i} = A{i};
end
for i= 1:num_sur-1
    C{i} = A{i};
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
for i = 1:num_sur
    Vx(i,:) = A{i}(2,:)-A{i}(1,:);
    Vx_norm=norm(Vx(i,:));
    Vx(i,:)=Vx(i,:)/Vx_norm;
    Vy(i,:)=cross(Vz(i,:),Vx(i,:));
end
for i = 1:num_sur -1
    cir_n(i,:) = cross(Vz(i,:),Vz(i+1,:));
end
for i =1:num_sur
R_sw(:,:,i)=[dot(Vx(i,:),Xa),dot(Vy(i,:),Xa),dot(Vz(i,:),Xa);dot(Vx(i,:),Ya),dot(Vy(i,:),Ya),dot(Vz(i,:),Ya);dot(Vx(i,:),Za),dot(Vy(i,:),Za),dot(Vz(i,:),Za)]; 
R_ws(:,:,i) = R_sw(:,:,i)';
end
for i=1:num_sur
    pw(i,:) = A{i}(1,:);
    psw(i,:) =(-R_ws(:,:,i)*pw(i,:)')';  %世界坐标系的原点在平面坐标系上的表示
end

for i=1:num_sur
    for j=1:num_point(i)
        B{i}(j,:) = (R_ws(:,:,i)*A{i}(j,:)')'+psw(i,:);
    end
end
ka = zeros(1,3);
for i = 1:num_sur-1
    nump = 1;
num_bor = size(B{i},1);
    max_borx = max(B{i}(:,1));      
    max_bory = max(B{i}(:,2));
    min_borx = min(B{i}(:,1));
    min_bory = min(B{i}(:,2));
ranx = max_borx-min_borx;
rany = max_bory-min_bory;
num_xx = ceil(ranx/reso_map);
num_yy = ceil(rany/reso_map);
wcvec = -[min_borx,min_bory,0];
for j=1:num_bor
    B{i}(j,:) =B{i}(j,:)+wcvec;
end 
for j =1:num_xx+1
    for k =1:num_yy+1
        map_poi = [(j-1)*reso_map, (k-1)*reso_map,0];
        if  insidepoly3(map_poi,B{i})  ==1
          tep  = (R_sw(:,:,i)*(map_poi -wcvec)')'+ pw(i,:);
          tep1 = tep + Vz(i,:)*L1;
          if circlepoly_inter(tep1,2*L2,cir_n(i,:),Aup{i+1})==1     
             C{i}(nump,:)  = (R_sw(:,:,i)*(map_poi -wcvec)')'+ pw(i,:);
             ka(nump,:) = tep;
             nump=nump+1;
         end
        end
     end
end

plot3(C{i}(:,1)*1000,C{i}(:,2)*1000,C{i}(:,3)*1000,'g*','LineWidth',1);
hold on;
end

end

