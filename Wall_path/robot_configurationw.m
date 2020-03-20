function [ trmatrix,jointangle ] = robot_configurationw(x1,Vn)
%根据求解的落足点
L1= 0.3407;L2=0.29325;Y=[0,1,0];
%[A,num_point,num_sur] = surfaces;
%[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map(map_border,obs_border,obs_height); 

num_sur = size(x1,1)-1;
% x1=zeros(num_sur+1,3);
% x1(:,1:2)=x;
% Vn=[0 0 1];%zeros(num_sur,3);
% for i=1:size(map_bor,1)
%     Vn=cross(map_bor(2,:)-map_bor(1,:),map_bor(3,:)-map_bor(2,:));   %计算多边形法向量,建立空间坐标系
%     Vn_norm=norm(Vn);
%     Vn=Vn/Vn_norm;
% end


Vz=Vn;
Vy=zeros(num_sur,3);
Vx=zeros(num_sur,3);
angle_sum=pi;
Ptr=zeros(num_sur+1,3);
angle=zeros(num_sur,3);
angle_ro=zeros(num_sur,1);
Ptar=zeros(num_sur,3);
trmatrix= zeros(4,4,num_sur);
jointangle = zeros(num_sur,5);

for i=1:num_sur
    trmatrix(:,:,i) = eye(4);
end

% for i=1:num_sur
%     Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
%     Vn_norm=norm(Vn(i,:));
%     Vn(i,:)=Vn(i,:)/Vn_norm;
% end
% for i=1:num_sur
%     for k = 1:num_point(i)
%         Pc(i,:)= Pc(i,:)+A{i}(k,:);
%     end
%    Pc(i,:) = Pc(i,:)/num_point(i);
% end
% for i=1:num_sur-1                                               %计算两多边形之间的坐标系单位方向向量
%     Vz(2*i-1,:)=Vn(i,:);                                
%     Vz(2*i,:)=Vn(i+1,:)*-1; 
% end
for i= 1:num_sur
    if i<=1
        
  % Vx(i,:)=([x1(2,1:2),0]-[x1(1,1:2),0])/norm([x1(2,1:2),0]-[x1(1,1:2),0]);
   Vx(i,:)=(x1(2,:)-x1(1,:))/norm(x1(2,:)-x1(1,:));
   lz = dot(Vx(i,:),Vn);
   Vx(i,:) =Vx(i,:)-lz*Vn;
   Vx(i,:) = Vx(i,:)/norm(Vx(i,:));
   else
        %Vx(i,:) =([x1(i,1:2),0]-[x1(i-1,1:2),0])/norm([x1(i,1:2),0]-[x1(i-1,1:2),0]) ;
        Vx(i,:)=(x1(i,:)-x1(i-1,:))/norm(x1(i,:)-x1(i-1,:));
        lz = dot(Vx(i,:),Vn);
        Vx(i,:) =Vx(i,:)-lz*Vn;
        Vx(i,:) = Vx(i,:)/norm(Vx(i,:));
   end
end

for i=1:num_sur
    Vy(i,:) = cross(Vz,Vx(i,:));
end
Vx1=(x1(num_sur+1,:)-x1(num_sur,:))/norm(x1(num_sur+1,:)-x1(num_sur,:));
lz = dot(Vx1,Vn);
Vx1=Vx1-lz*Vn;
Vx1 = Vx1/norm(Vx1);

Vy1=cross(Vz,Vx1);
% for i= 1:num_sur-1                            %计算两多边形之间的转角 
%    xx=dot(Vx(2*i,:),Vx(2*i-1,:));
%    yy=dot(Vx(2*i,:),Vz(2*i-1,:));
%    angle_sum(i)=atan2(yy,xx);
% end
for i=2:num_sur
    if i==num_sur
        angle_ro(i) = atan2(dot(Vx1,Vy(i,:)),dot(Vx1,Vx(i,:)))*180/pi;
    else
    angle_ro(i) = atan2(dot(Vx(i+1,:),Vy(i,:)),dot(Vx(i+1,:),Vx(i,:)))*180/pi;
    end
end
for i= 1:num_sur+1                            %计算平移后的点的位置 
   Ptr(i,:) = x1(i,:)+Vn*L1;        
  % Ptr(2*i,:) = x(2*i,:)+Vn(i+1,:)*L1;
end

for i= 1:num_sur                           %计算目标吸附点在基座标系下的表示
    if i ==num_sur
        
        Ptar(i,:)= [dot(Ptr(i+1,:)-Ptr(i,:),Vx1), dot(Ptr(i+1,:)-Ptr(i,:),Vy1), dot(Ptr(i+1,:)-Ptr(i,:),Vz)];
    else
  Ptar(i,:) = [dot(Ptr(i+1,:)-Ptr(i,:),Vx(i+1,:)), dot(Ptr(i+1,:)-Ptr(i,:),Vy(i+1,:)), dot(Ptr(i+1,:)-Ptr(i,:),Vz)];
    end
end
 for i= 1:num_sur
     angle(i,1) = acos((Ptar(i,1)^2+Ptar(i,3)^2)/(2*L2*sqrt(Ptar(i,1)^2+Ptar(i,3)^2)))+atan2(Ptar(i,3),Ptar(i,1));    %计算逆运动学的三个转角
     angle(i,2) = -acos((Ptar(i,1)^2+Ptar(i,3)^2-2*L2^2)/(2*L2^2));
     angle(i,3) = angle_sum - angle(i,2) - angle(i,1) +1.57; 
     angle(i,:) = angle(i,:)*180/pi;
 end
 
 for i=1:num_sur
     jointangle(i,2:4)= angle(i,:);
     jointangle(i,1) = angle_ro(i);
 end
for i=1:num_sur
%     if i==num_sur
%         trmatrix(1:3,1,i)= Vx1';
%     trmatrix(1:3,2,i)= Vy1';
%     else
    trmatrix(1:3,1,i)= Vx(i,:)';
    trmatrix(1:3,2,i)= Vy(i,:)';
%     end
    trmatrix(1:3,3,i)= Vz';
    trmatrix(1:3,4,i)=  x1(i,:)';
end

end

