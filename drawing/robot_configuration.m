function [ trmatrix,jointangle ] = robot_configuration(x)
%根据起始吸附点和目标吸附点计算机器人的基点坐标系和各个关节角
L1= 0.3407;L2=0.29325;Y=[0,1,0];
[A,num_point,num_sur] = surfaces;
Vn=zeros(num_sur,3);
Pc=zeros(num_sur,3);
Vz=zeros((num_sur-1)*2,3);
Vy=zeros((num_sur-1)*2,3);
Vx=zeros((num_sur-1)*2,3);
angle_sum=zeros(num_sur-1,1);
Ptr=zeros((num_sur-1)*2,3);
angle=zeros(num_sur-1,3);
Ptar=zeros(num_sur-1,3);
trmatrix= zeros(4,4,num_sur-1);

jointangle = zeros(num_sur-1,5);
for i=1:num_sur-1
    trmatrix(:,:,i) = eye(4);
end
for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm;
end
for i=1:num_sur
    for k = 1:num_point(i)
        Pc(i,:)= Pc(i,:)+A{i}(k,:);
    end
   Pc(i,:) = Pc(i,:)/num_point(i);
end
for i=1:num_sur-1                                               %计算两多边形之间的坐标系单位方向向量
    Vz(2*i-1,:)=Vn(i,:);                                
    Vz(2*i,:)=Vn(i+1,:)*-1; 
end
for i=1:num_sur-1
    Vy(2*i-1,:)=cross(Vz(2*i-1,:),Vz(2*i,:));
    if dot(Vy(2*i-1,:),Y)<0
    Vy(2*i-1,:)= -Vy(2*i-1,:);
    end
    Vy_norm=norm(Vy(2*i-1,:));
    Vy(2*i-1,:)=Vy(2*i-1,:)/Vy_norm;
    Vy(2*i,:) =  Vy(2*i-1,:);
end
for i= 1:num_sur-1
    Vx(2*i-1,:)=cross(Vy(2*i-1,:),Vz(2*i-1,:));
    Vx(2*i,:)=cross(Vy(2*i,:),Vz(2*i,:));
end
for i= 1:num_sur-1                            %计算两多边形之间的转角 
   xx=dot(Vx(2*i,:),Vx(2*i-1,:));
   yy=dot(Vx(2*i,:),Vz(2*i-1,:));
   angle_sum(i)=atan2(yy,xx);
end
for i= 1:num_sur-1                            %计算平移后的点的位置 
   Ptr(2*i-1,:) = x(2*i-1,:)+Vn(i,:)*L1;        
   Ptr(2*i,:) = x(2*i,:)+Vn(i+1,:)*L1;
end
for i= 1:num_sur-1                           %计算目标吸附点在基座标系下的表示
  Ptar(i,:) = [dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vx(2*i-1,:)), dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vy(2*i-1,:)), dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vz(2*i-1,:))];
end
 for i= 1:num_sur-1
     angle(i,1) = acos((Ptar(i,1)^2+Ptar(i,3)^2)/(2*L2*sqrt(Ptar(i,1)^2+Ptar(i,3)^2)))+atan2(Ptar(i,3),Ptar(i,1));    %计算逆运动学的三个转角
     angle(i,2) = -acos((Ptar(i,1)^2+Ptar(i,3)^2-2*L2^2)/(2*L2^2));
     angle(i,3) = angle_sum(i) - angle(i,2) - angle(i,1) +1.57; 
     angle(i,:) = angle(i,:)*180/pi;
 end
 
 for i=1:num_sur-1
     jointangle(i,2:4)= angle(i,:);
 end
 
for i=1:num_sur-1
    trmatrix(1:3,1,i)= Vx(2*i-1,:)';
    trmatrix(1:3,2,i)= Vy(2*i-1,:)';
    trmatrix(1:3,3,i)= Vz(2*i-1,:)';
    trmatrix(1:3,4,i)=  x(2*i-1,:)';
end

end
% x=76532
%     4.4499    6.7528    1.7845
%     4.8465    6.8204    2.5235
%     8.1729    6.0005    5.1687
%     7.7191    5.2131    6.0000
%     7.7195    4.3677    5.5540
%     8.2811    3.9256    5.6526
%      7.8746    3.7953    8.7276
%     7.2444    4.3322    9.3113
