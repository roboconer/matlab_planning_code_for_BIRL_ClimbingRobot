 function ThermalEnergy=optimalpath(z1,z2,alpha)
tt=24;
n=20;       %%曲线分的段数
td=[3/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 3/(n+4)]*tt;    %%每段期望运行时间和用于计算加速度的矩阵
M=inv([2 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;3 8 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
       0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
       0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0;
       0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0;
       0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0;
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0;
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/3 8/3 1;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 2]);


%    z1=-650;z2=-650;alpha=1.41;
% z1=optz1;
% z2=optz2;
% alpha=optalpha;
%%%%%%%%%%%%初始位置
xg=200;yg=0;zg=-1100;xs=-720;ys=0;zs=-80; 
%xg=-400;yg=500;zg=-80;xs=-720;ys=0;zs=-80;
ps=[xs;ys;zs];pg=[xg;yg;zg];

Rs=[1 0 0;0 1 0; 0 0 1];
Rg=[0 0 1; 0 1 0; -1 0 0];

n=20;       %%曲线分的段数
%%%%%%%%%%%%给定动力学约束
maxacc1=0.52;maxvel1=0.7;maxacc2=0.52;maxvel2=0.7;maxacc3=0.52;maxvel3=0.7;maxacc4=0.52;maxvel4=0.7;maxacc5=0.52;maxvel5=0.7;%各轴最大速度和加速度

s=((xg-xs)^2+(yg-ys)^2+(zg-zs)^2)^0.5;
input=[z1,z2,s];
[A,B,C]=getspline(input);
for i=1:(n+1)
    x(i)=(i-1)*s/n;
    z(i)=A*(x(i))^3+B*(x(i))^2+C*x(i);               %%求出路径点在规划平面内的参考坐标系下的坐标
    p(:,:,i)=[x(i);z(i)*sin(alpha);z(i)*cos(alpha)];      %%转化为三维坐标
    A1=atan2((yg-ys),(xg-xs)); A2=atan2(-(zg-zs),((xg-xs)^2+(yg-ys)^2)^0.5);
    po(:,:,i)=[cos(A1) -sin(A1) 0;sin(A1) cos(A1) 0; 0 0 1]*[cos(A2) 0 sin(A2); 0 1 0; -sin(A2) 0 cos(A2)]*[x(i);z(i)*sin(alpha);z(i)*cos(alpha)]; %%转化到基坐标系中
    x(i)=po(1,1,i)+xs; y(i)=po(2,1,i)+ys;z(i)=po(3,1,i)+zs;
%%%%%%%%%%%%%%%计算路径点位置  5轴机器人 扭转步态
    ay=-1*pi/2;                                         %%取y轴插补角度
    az=0;                                         %%取z轴插补角度
    px=x(i);py=y(i);pz=z(i);
    Theta1(i)=atan2(-py,-px);
%     Theta234(i)=atan2(r13*cos(a1)+r23*sin(a1), r33);
    R(:,:,i)=[cos(Theta1(i)) -sin(Theta1(i)) 0;sin(Theta1(i)) cos(Theta1(i)) 0; 0 0 1]*[cos((i-1)*ay/n) 0 sin((i-1)*ay/n); 0 1 0; -sin((i-1)*ay/n) 0 cos((i-1)*ay/n)];
    r11=R(1,1,i);r12=R(1,2,i);r13=R(1,3,i);r21=R(2,1,i);r22=R(2,2,i);r23=R(2,3,i);r31=R(3,1,i);r32=R(3,2,i);r33=R(3,3,i);
    [a1,a2,a3,a4,a5]=invkinematics_around(r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz);
    Theta1(i)=a1; Theta2(i)=a2; Theta3(i)=a3; Theta4(i)=a4; Theta5(i)=a5;
    if abs(abs(Theta1(i))-pi)<=10^(-5)
        if Theta1(i-1)>0
            Theta1(i)=pi;
        end
        if Theta1(i-1)<0
            Theta1(i)=-pi;
        end
    end
 %%%%%%%%%%%%%%%  插补姿态  5轴机器人 翻转步态 
%  ay=-3*pi/2;                                         %%取y轴插补角度
%     az=0;                                         %%取z轴插补角度
%     px=x(i);py=y(i);pz=z(i);
%     if px<=0   
%        Theta1(i)=atan2(-py,-px);
%        elseif px>0&&py>=0
%               Theta1(i)=atan2(-py,-px)+pi;
%     else
%         Theta1(i)=atan2(-py,-px)-pi;
%     end
%     R(:,:,i)=[cos(Theta1(i)) -sin(Theta1(i)) 0;sin(Theta1(i)) cos(Theta1(i)) 0; 0 0 1]*[cos((i-1)*ay/n) 0 sin((i-1)*ay/n); 0 1 0; -sin((i-1)*ay/n) 0 cos((i-1)*ay/n)];
%     r11=R(1,1,i);r12=R(1,2,i);r13=R(1,3,i);r21=R(2,1,i);r22=R(2,2,i);r23=R(2,3,i);r31=R(3,1,i);r32=R(3,2,i);r33=R(3,3,i);
%     [a1,a2,a3,a4,a5]=invkinematics_over(r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz);
%     Theta1(i)=a1; Theta2(i)=a2; Theta3(i)=a3; Theta4(i)=a4; Theta5(i)=a5;
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%运动学约束
if max(Theta2(:))>10^5
    ThermalEnergy=10^30;
    return
else    
% %%%%%%%%%%%%%%%%%%%%%%%计算路径点速度(用于给定中间速度的方法）
% for i=1:n
%     if i==1
%        vel1(i)=sign(Theta1(i)-Theta01)*maxvel1/2;
%        vel2(i)=sign(Theta2(i)-Theta02)*maxvel1/2;
%        vel3(i)=sign(Theta3(i)-Theta03)*maxvel1/2;
%        vel4(i)=sign(Theta4(i)-Theta04)*maxvel1/2;
%        vel5(i)=sign(Theta5(i)-Theta05)*maxvel1/2;
%     else
%        vel1(i)=sign(Theta1(i)-Theta1(i-1))*maxvel1/2;
%        vel2(i)=sign(Theta2(i)-Theta2(i-1))*maxvel1/2;
%        vel3(i)=sign(Theta3(i)-Theta3(i-1))*maxvel1/2;
%        vel4(i)=sign(Theta4(i)-Theta4(i-1))*maxvel1/2;
%        vel5(i)=sign(Theta5(i)-Theta5(i-1))*maxvel1/2;
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%计算n-1个路劲点加速度、速度（用于加速度连续的方法）
for i=1:n+1
    if i==1
       matrix1(i)=6*(-Theta1(i)+Theta1(i+1))/td(i)/td(i);
       matrix2(i)=6*(-Theta2(i)+Theta2(i+1))/td(i)/td(i);
       matrix3(i)=6*(-Theta3(i)+Theta3(i+1))/td(i)/td(i);
       matrix4(i)=6*(-Theta4(i)+Theta4(i+1))/td(i)/td(i);
       matrix5(i)=6*(-Theta5(i)+Theta5(i+1))/td(i)/td(i);
       elseif i==n+1
       matrix1(i)=-6*(Theta1(i)-Theta1(i-1))/td(i-1)/td(i-1);
       matrix2(i)=-6*(Theta2(i)-Theta2(i-1))/td(i-1)/td(i-1);
       matrix3(i)=-6*(Theta3(i)-Theta3(i-1))/td(i-1)/td(i-1);
       matrix4(i)=-6*(Theta4(i)-Theta4(i-1))/td(i-1)/td(i-1);
       matrix5(i)=-6*(Theta5(i)-Theta5(i-1))/td(i-1)/td(i-1);    
    else    
    matrix1(i)=((Theta1(i+1)-Theta1(i))/td(i)-(Theta1(i)-Theta1(i-1))/td(i-1))*6/td(i); 
    matrix2(i)=((Theta2(i+1)-Theta2(i))/td(i)-(Theta2(i)-Theta2(i-1))/td(i-1))*6/td(i);
    matrix3(i)=((Theta3(i+1)-Theta3(i))/td(i)-(Theta3(i)-Theta3(i-1))/td(i-1))*6/td(i);
    matrix4(i)=((Theta4(i+1)-Theta4(i))/td(i)-(Theta4(i)-Theta4(i-1))/td(i-1))*6/td(i);
    matrix5(i)=((Theta5(i+1)-Theta5(i))/td(i)-(Theta5(i)-Theta5(i-1))/td(i-1))*6/td(i);
    end
end
acc1=M*(matrix1.');
acc2=M*(matrix2.');
acc3=M*(matrix3.');
acc4=M*(matrix4.');
acc5=M*(matrix5.');
vel1(n+1)=0;vel2(n+1)=0;vel3(n+1)=0;vel4(n+1)=0;vel5(n+1)=0;
for i=1:n
    vel1(i+1)=vel1(i)+(acc1(i+1)+acc1(i))*td(i)/2;
    vel2(i+1)=vel2(i)+(acc2(i+1)+acc2(i))*td(i)/2;
    vel3(i+1)=vel3(i)+(acc3(i+1)+acc3(i))*td(i)/2;
    vel4(i+1)=vel4(i)+(acc4(i+1)+acc4(i))*td(i)/2;
    vel5(i+1)=vel5(i)+(acc5(i+1)+acc5(i))*td(i)/2;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%求每一段路径中点位置、速度、加速度
for i=1:n
    Macc1(i)=(acc1(i)+acc1(i+1))/2;
    Macc2(i)=(acc2(i)+acc2(i+1))/2;
    Macc3(i)=(acc3(i)+acc3(i+1))/2;
    Macc4(i)=(acc4(i)+acc4(i+1))/2;
    Macc5(i)=(acc5(i)+acc5(i+1))/2;
    Mvel1(i)=vel1(i)+(acc1(i+1)+3*acc1(i))*td(i)/8;
    Mvel2(i)=vel2(i)+(acc2(i+1)+3*acc2(i))*td(i)/8;
    Mvel3(i)=vel3(i)+(acc3(i+1)+3*acc3(i))*td(i)/8;
    Mvel4(i)=vel4(i)+(acc4(i+1)+3*acc4(i))*td(i)/8;
    Mvel5(i)=vel5(i)+(acc5(i+1)+3*acc5(i))*td(i)/8;
    Mpos1(i)=Theta1(i)+vel1(i)*td(i)/2+(acc1(i+1)-acc1(i))/48*td(i)*td(i)+acc1(i)*td(i)*td(i)/8;
    Mpos2(i)=Theta2(i)+vel2(i)*td(i)/2+(acc2(i+1)-acc2(i))/48*td(i)*td(i)+acc2(i)*td(i)*td(i)/8;
    Mpos3(i)=Theta3(i)+vel3(i)*td(i)/2+(acc3(i+1)-acc3(i))/48*td(i)*td(i)+acc3(i)*td(i)*td(i)/8;
    Mpos4(i)=Theta4(i)+vel4(i)*td(i)/2+(acc4(i+1)-acc4(i))/48*td(i)*td(i)+acc4(i)*td(i)*td(i)/8;
    Mpos5(i)=Theta5(i)+vel5(i)*td(i)/2+(acc5(i+1)-acc5(i))/48*td(i)*td(i)+acc5(i)*td(i)*td(i)/8;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%动力学约束
if max(abs(acc1(:)))>maxacc1||max(abs(acc2(:)))>maxacc2||max(abs(acc3(:)))>maxacc3||max(abs(acc4(:)))>maxacc4||max(abs(acc5(:)))>maxacc5||max(abs(vel1(:)))>maxvel1||max(abs(vel2(:)))>maxvel2||max(abs(vel3(:)))>maxvel3||max(abs(vel4(:)))>maxvel4||max(abs(vel5(:)))>maxvel5||max(abs(Mvel1(:)))>maxvel1||max(abs(Mvel2(:)))>maxvel2||max(abs(Mvel3(:)))>maxvel3||max(abs(Mvel4(:)))>maxvel4||max(abs(Mvel5(:)))>maxvel5
    ThermalEnergy=10^30;
    return
else
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%计算路径中点力矩 牛顿欧拉迭代
for i=1:n
    a=[0 Mpos1(i) Mpos2(i) Mpos3(i) Mpos4(i) Mpos5(i) 0];
    aa=[0 Mvel1(i) Mvel2(i) Mvel3(i) Mvel4(i) Mvel5(i) 0];
    aaa=[0 Macc1(i) Macc2(i) Macc3(i) Macc4(i) Macc5(i) 0];
    [Torque1,Torque2,Torque3,Torque4,Torque5]=gettorque(a,aa,aaa);
    tor1(i)=Torque1;
    tor2(i)=Torque2;
    tor3(i)=Torque3;
    tor4(i)=Torque4;
    tor5(i)=Torque5;
end

%%%%%%%%%%%%%%%%%计算能量的等效式
ThermalEnergy=0;
for i=1:n
J=tor1(i)^2*td(i)+tor2(i)^2*td(i)+tor3(i)^2*td(i)+tor4(i)^2*td(i)+tor5(i)^2*td(i);
ThermalEnergy=J+ThermalEnergy;
end
end
end




% view(3);                                           %%检测矩阵变换是否正确
% grid on;
% axis([-1500 1500 -1500 1500 -1500 1500]);
% hold on;
% x=[x];y=[y];z=[z];
% % line([ps(3),pg(3)],[ps(2),pg(2)],[ps(1),pg(1)]);
% plot3(z,y,x,'g.');
   