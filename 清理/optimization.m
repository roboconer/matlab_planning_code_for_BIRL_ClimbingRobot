function [f,ceq] = optimization(x)
L1= 0.3407;L2=0.29325;
Ps1=[0.3,0.2,0;4,0.1,0;3.5,4,0;1,3,0];
Ps2=[0.2,0.1,0.2;0.5,3.3,0.2;0.7,1.4,3];
Ps3=[0.8,0.3,2;1,1.7,3.8;3,1.3,4;2.7,0.5,2.90694];
Y=[0,1,0];
Vn1=[0,0,1];
Vn2=[0.986755 -0.0925083 -0.133256];
Vn3=cross(Ps3(2,:)-Ps3(1,:),Ps3(3,:)-Ps3(2,:));   %计算多边形法向量和中心点
Vn3_norm=norm(Vn3);
Vn3=Vn3/Vn3_norm;
Pc1=[2.2 1.825 0];
Pc2=[7/15 1.6 17/15];
Pc3=[0,0,0];
size3= size(Ps3,1);
for i=1:size3
    Pc3=Pc3+Ps3(i,:);
end
Pc3=Pc3/size3;
Vy1=[0.0933407 0.995634 0];
Vx1=[0.995634 -0.0933407 0];
Vz3=Vn2;                   %计算两多边形之间的坐标系单位方向向量
Vz4=-Vn3;
Vy3=cross(Vz3,Vz4);
if dot(Vy3,Y)<0
    Vy3= -Vy3;
end
Vy4 = Vy3;
Vx3=cross(Vy3,Vz3);
Vx4=cross(Vy4,Vz4);
x2=dot(Vx4,Vx3);
y2=dot(Vx4,Vz3);
angle_sum=1.43714;
angle_sum2=atan2(y2,x2);        %计算两多边形之间的转角 
Ptr1 = x(1,:)+Vn1*L1;        
Ptr2 = x(2,:)+Vn2*L1;
Ptr3 = x(3,:)+Vn2*L1;
Ptr4 = x(4,:)+Vn3*L1;
Ptar = [dot(Ptr2-Ptr1,Vx1), dot(Ptr2-Ptr1,Vy1), dot(Ptr2-Ptr1,Vn1)]; 
Ptar2 = [dot(Ptr4-Ptr3,Vx3),dot(Ptr4-Ptr3,Vy3), dot(Ptr4-Ptr3,Vz3)];     %计算目标吸附点在基座标系下的表示
angle1 = -acos((Ptar(1)^2+Ptar(3)^2-2*L2^2)/(2*L2));
angle2 = acos((Ptar(1)^2+Ptar(3)^2)/(2*L2*sqrt(Ptar(1)^2+Ptar(3)^2)))+atan2(Ptar(3),Ptar(1));
angle3 = angle_sum - angle2 - angle1 +1.57;
angle4 = -acos((Ptar2(1)^2+Ptar2(3)^2-2*L2^2)/(2*L2));        %计算逆运动学的三个转角
angle5 = acos((Ptar2(1)^2+Ptar2(3)^2)/(2*L2*sqrt(Ptar2(1)^2+Ptar2(3)^2)))+atan2(Ptar2(3),Ptar2(1));
angle6 = angle_sum2 - angle4 - angle5 +1.57;
ceq(1)=dot((x(1,:)-Pc1),Vn1);                              %等式约束，点在多边形所在平面上
ceq(2)=dot((x(2,:)-Pc2),Vn2);
ceq(3)=dot((x(3,:)-Pc2),Vn2);                             
ceq(4)=dot((x(4,:)-Pc3),Vn3);
f(1)=-dot(cross(x(1,:)-Ps1(1,:),Ps1(2,:)-Ps1(1,:)),cross(Pc1-Ps1(1,:),Ps1(2,:)-Ps1(1,:)));            %不等式约束，点在多边形边界内
f(2)=-dot(cross(x(1,:)-Ps1(2,:),Ps1(3,:)-Ps1(2,:)),cross(Pc1-Ps1(2,:),Ps1(3,:)-Ps1(2,:)));
f(3)=-dot(cross(x(1,:)-Ps1(3,:),Ps1(4,:)-Ps1(3,:)),cross(Pc1-Ps1(3,:),Ps1(4,:)-Ps1(3,:)));
f(4)=-dot(cross(x(1,:)-Ps1(4,:),Ps1(1,:)-Ps1(4,:)),cross(Pc1-Ps1(4,:),Ps1(1,:)-Ps1(4,:)));
f(5)=-dot(cross(x(2,:)-Ps2(1,:),Ps2(2,:)-Ps2(1,:)),cross(Pc2-Ps2(1,:),Ps2(2,:)-Ps2(1,:)));
f(6)=-dot(cross(x(2,:)-Ps2(2,:),Ps2(3,:)-Ps2(2,:)),cross(Pc2-Ps2(2,:),Ps2(3,:)-Ps2(2,:)));
f(7)=-dot(cross(x(2,:)-Ps2(3,:),Ps2(1,:)-Ps2(3,:)),cross(Pc2-Ps2(3,:),Ps2(1,:)-Ps2(3,:)));
f(8)=-dot(cross(x(3,:)-Ps2(1,:),Ps2(2,:)-Ps2(1,:)),cross(Pc2-Ps2(1,:),Ps2(2,:)-Ps2(1,:)));
f(9)=-dot(cross(x(3,:)-Ps2(2,:),Ps2(3,:)-Ps2(2,:)),cross(Pc2-Ps2(2,:),Ps2(3,:)-Ps2(2,:)));
f(10)=-dot(cross(x(3,:)-Ps2(3,:),Ps2(1,:)-Ps2(3,:)),cross(Pc2-Ps2(3,:),Ps2(1,:)-Ps2(3,:)));
f(11)=-dot(cross(x(4,:)-Ps3(1,:),Ps3(2,:)-Ps3(1,:)),cross(Pc3-Ps3(1,:),Ps3(2,:)-Ps3(1,:)));            %不等式约束，点在多边形边界内
f(12)=-dot(cross(x(4,:)-Ps3(2,:),Ps3(3,:)-Ps3(2,:)),cross(Pc3-Ps3(2,:),Ps3(3,:)-Ps3(2,:)));
f(13)=-dot(cross(x(4,:)-Ps3(3,:),Ps3(4,:)-Ps3(3,:)),cross(Pc3-Ps3(3,:),Ps3(4,:)-Ps3(3,:)));
f(14)=-dot(cross(x(4,:)-Ps3(4,:),Ps3(1,:)-Ps3(4,:)),cross(Pc3-Ps3(4,:),Ps3(1,:)-Ps3(4,:)));


f(15)=-(Ptar(1)^2+Ptar(3)^2-2*L2^2)/(2*L2)-0.5;                             %三个转角不等式约束
f(16)=-sin(acos((Ptar(1)^2+Ptar(3)^2)/(2*L2*sqrt(Ptar(1)^2+Ptar(3)^2)))+atan2(Ptar(3),Ptar(1)))-0.866;
f(17)=-cos(angle3)-0.5;
f(18)=-cos(angle4)-0.5;                             %三个转角不等式约束
f(19)=-sin(angle5)-0.866;
f(20)=-cos(angle6)-0.5;
f(21)=norm(Ptr2-Ptr1)-2*L2;             %两个过渡吸附点之间的位置约束
ceq(5)=dot(Ptr2-Ptr1,Vy1);
f(22)=norm(Ptr4-Ptr3)-2*L2;             %两个过渡吸附点之间的位置约束
ceq(6)=dot(Ptr4-Ptr3,Vy3);
%ceq(1)=x(1,3);
end
