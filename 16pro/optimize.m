function [f,ceq] = optimize(x)
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
B= cell((num_sur-1)*2,1);        %多边形点进行坐标系转换后的储存
Vn2=zeros((num_sur-1)*2,3);       %多边形转换后的法向量
Pc2=zeros((num_sur-1)*2,3);       %多边形转换后的中心点
lin = cell(num_sur-1,1);        %连杆坐标点
jud = zeros((num_sur-1)*2,3);    %连杆与多边形相交判断系数
num_ceq=0;num_f=0;

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
 end
for i=1:num_sur-1
    ceq(2*i-1)=dot(x(2*i-1,:)-Pc(i,:),Vn(i,:));                              %等式约束，点在多边形所在平面上   
    ceq(2*i)=dot(x(2*i,:)-Pc(i+1,:),Vn(i+1,:));
end
num_ceq=num_ceq+ 2*(num_sur-1);
for i=1:num_sur-1
    for k=1:num_point(i)
        if k ==num_point(i)
            f(num_f+k)=-dot(cross(x(2*i-1,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)));
        else 
            f(num_f+k)=-dot(cross(x(2*i-1,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)));            %不等式约束，点在多边形边界内
        end
    end
num_f = num_f+num_point(i);
end
for i=2:num_sur
    for k= 1:num_point(i)
        if k==num_point(i)
          f(num_f+k)=-dot(cross(x(2*i-2,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)));
          else 
            f(num_f+k)=-dot(cross(x(2*i-2,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)));            %不等式约束，点在多边形边界内
        end
    end
    num_f = num_f+num_point(i);
end
for i=1:num_sur-1
    f(num_f+1)=-sin(angle(i,1))-0.866;                             %三个转角不等式约束
   f(num_f+2)=-cos(angle(i,2))-0.5;
   f(num_f+3)=-cos(angle(i,3))-0.5;
   num_f = num_f+3;
end
for i=1:num_sur-1
    f(num_f+i)=norm(Ptar(i,:))-2*L2+0.001;             %两个过渡吸附点之间的位置约束
    ceq(num_ceq+i)=dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vy(2*i-1,:));
end
num_f = num_f + num_sur-1;
num_ceq = num_ceq + num_sur-1;
for i=1:num_sur-1                                    %对转换后的点坐标初始化
    B{2*i-1}=zeros(size(A{i},1),size(A{i},2));
    B{2*i}=zeros(size(A{i+1},1),size(A{i+1},2));
end
for i=1:num_sur-1                            %求坐标转换后的点坐标
    for k=1:num_point(i)
        B{2*i-1}(k,:) = [dot(A{i}(k,:)-x(2*i-1,:),Vx(2*i-1,:)),dot(A{i}(k,:)-x(2*i-1,:),Vy(2*i-1,:)),dot(A{i}(k,:)-x(2*i-1,:),Vz(2*i-1,:))];
    end
    for k=1:num_point(i+1)
         B{2*i}(k,:) = [dot(A{i+1}(k,:)-x(2*i-1,:),Vx(2*i-1,:)),dot(A{i+1}(k,:)-x(2*i-1,:),Vy(2*i-1,:)),dot(A{i+1}(k,:)-x(2*i-1,:),Vz(2*i-1,:))];
    end
end

for i=1:num_sur-1                     %求坐标转换后的多边形的法向量和中心点
     Vn2(2*i-1,:)=cross(B{2*i-1}(2,:)-B{2*i-1}(1,:),B{2*i-1}(3,:)-B{2*i-1}(2,:));  
    Vn_norm=norm(Vn2(2*i-1,:));
    Vn2(2*i-1,:)=Vn2(2*i-1,:)/Vn_norm;
     Vn2(2*i,:)=cross(B{2*i}(2,:)-B{2*i}(1,:),B{2*i}(3,:)-B{2*i}(2,:));  
    Vn_norm=norm(Vn2(2*i,:));
    Vn2(2*i,:)=Vn2(2*i,:)/Vn_norm;
end
for i=1:num_sur-1
    for k = 1:num_point(i)
        Pc2(2*i-1,:)= Pc2(2*i-1,:)+B{2*i-1}(k,:);
    end
   Pc2(2*i-1,:) = Pc2(2*i-1,:)/num_point(i);
    for k = 1:num_point(i+1)
        Pc2(2*i,:)= Pc2(2*i,:)+B{2*i}(k,:);
    end
   Pc2(2*i,:) = Pc2(2*i,:)/num_point(i+1);
end
for i=1:num_sur-1
    lin{i}= zeros(5,3);
end
 for i=num_sur-1                                    %求机器人连杆坐标
      lin{i}(1,:) = [0 0 0];
      lin{i}(2,:) = [0 0 L1];
      lin{i}(3,:) = [L2*cos(angle(i,1)),0, L1+L2*sin(angle(i,1))];
      lin{i}(4,:) = [lin{i}(3,1)+L2*cos(angle(i,1)+angle(i,2)),0,lin{i}(3,3)+L2*sin(angle(i,1)+angle(i,2))];
      lin{i}(5,:) = [lin{i}(4,1)+L1*cos(angle(i,1)+angle(i,2)+angle(i,3)),0,lin{i}(4,3)+L1*sin(angle(i,1)+angle(i,2)+angle(i,3))];
 end
for i= 1:num_sur-1
    for k= 2:4
       vec_link = lin{i}(k+1,:)-lin{i}(k,:);
       t_coe = dot(Vn2(2*i-1,:),B{2*i-1}(1,:)-lin{i}(k,:))/dot(Vn2(2*i-1,:),vec_link);
       inter_sec = lin{i}(k,:) + t_coe*vec_link;                                      %求连杆线段与多边形平面交点
       if t_coe>1||t_coe<0
           jud(2*i-1,k-1) = num_point(i);
       else
           for j=1:num_point(i)
            if j ==num_point(i)
           num_jud = dot(cross(inter_sec-B{2*i-1}(j,:),B{2*i-1}(1,:)-B{2*i-1}(j,:)),cross(Pc2(2*i-1,:)-B{2*i-1}(j,:),B{2*i-1}(1,:)-B{2*i-1}(j,:)));
        else 
            num_jud = dot(cross(inter_sec-B{2*i-1}(j,:),B{2*i-1}(j+1,:)-B{2*i-1}(j,:)),cross(Pc2(2*i-1,:)-B{2*i-1}(j,:),B{2*i-1}(j+1,:)-B{2*i-1}(j,:)));            %点不在多边形边界内
            end 
            if num_jud <0
              jud(2*i-1,k-1)= jud(2*i-1,k-1)+1;
            end
           end
       end
    end
    for k = 1:3
      vec_link = lin{i}(k+1,:)-lin{i}(k,:);
       t_coe = dot(Vn2(2*i,:),B{2*i}(1,:)-lin{i}(k,:))/dot(Vn2(2*i,:),vec_link);
       inter_sec = lin{i}(k,:) + t_coe*vec_link;                                  %求连杆线段与多边形平面交点
       if t_coe>1||t_coe<0
           jud(2*i,k) = num_point(i+1);
       else
           for j=1:num_point(i+1)
            if j ==num_point(i+1)
           num_jud = dot(cross(inter_sec-B{2*i}(j,:),B{2*i}(1,:)-B{2*i}(j,:)),cross(Pc2(2*i,:)-B{2*i}(j,:),B{2*i}(1,:)-B{2*i}(j,:)));
        else 
            num_jud = dot(cross(inter_sec-B{2*i}(j,:),B{2*i}(j+1,:)-B{2*i}(j,:)),cross(Pc2(2*i,:)-B{2*i}(j,:),B{2*i}(j+1,:)-B{2*i}(j,:)));            %点不在多边形边界内
            end 
            if num_jud <0
              jud(2*i,k)= jud(2*i,k)+1;
            end
           end
       end
    end   
end
 for i= 1:(num_sur-1)*2
    f(num_f+1)=-jud(i,1)+1;
    f(num_f+2)=-jud(i,2)+1;
    f(num_f+3)=-jud(i,3)+1;
     num_f=num_f+3;
 end
end
