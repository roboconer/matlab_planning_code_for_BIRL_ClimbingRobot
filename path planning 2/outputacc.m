 function [acc1,acc2,acc3,acc4,acc5,vel1,vel2,vel3,vel4,vel5,Theta1,Theta2,Theta3,Theta4,Theta5,td,n,tor1,tor2,tor3,tor4,tor5]=outputacc(optz1,optz2,optalpha)
tt=24;    %�涨�˶�ʱ��Ϊ24s
n=20;       %%���߷ֵĶ���
td=[3/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 1/(n+4) 3/(n+4)]*tt;      %%ÿ����������ʱ������ڼ�����ٶȵľ���
M=inv([2 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;3 8 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
       0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
       0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0 0;
       0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0 0;
       0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0 0;
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0 0;
       0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 4 1 0;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1/3 8/3 1;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 2]);
z1=optz1;
z2=optz2;
alpha=optalpha;
%%%%%%%%%%%%��ʼλ��
xg=200;yg=0;zg=-1100;xs=-720;ys=0;zs=-80; 
%xg=-400;yg=500;zg=-80;xs=-720;ys=0;zs=-80;
ps=[xs;ys;zs];pg=[xg;yg;zg];

Rs=[1 0 0;0 1 0; 0 0 1];
Rg=[0 0 1; 0 1 0; -1 0 0];

%%%%%%%%%%%%��������ѧԼ��
% maxacc1=30;maxvel1=30;maxacc2=30;maxvel2=30;maxacc3=30;maxvel3=30;maxacc4=30;maxvel4=30;maxacc5=30;maxvel5=30;

s=((xg-xs)^2+(yg-ys)^2+(zg-zs)^2)^0.5;
input=[z1,z2,s];
[A,B,C]=getspline(input);
for i=1:(n+1)
    x(i)=(i-1)*s/n;
    z(i)=A*(x(i))^3+B*(x(i))^2+C*x(i);               %%���·�����ڹ滮ƽ���ڵĲο�����ϵ�µ�����
    p(:,:,i)=[x(i);z(i)*sin(alpha);z(i)*cos(alpha)];      %%ת��Ϊ��ά����
    A1=atan2((yg-ys),(xg-xs)); A2=atan2(-(zg-zs),((xg-xs)^2+(yg-ys)^2)^0.5);
    po(:,:,i)=[cos(A1) -sin(A1) 0;sin(A1) cos(A1) 0; 0 0 1]*[cos(A2) 0 sin(A2); 0 1 0; -sin(A2) 0 cos(A2)]*[x(i);z(i)*sin(alpha);z(i)*cos(alpha)]; %%ת����������ϵ��
    x(i)=po(1,1,i)+xs; y(i)=po(2,1,i)+ys;z(i)=po(3,1,i)+zs;
%%%%%%%%%%%%%%%����·����λ��  5������� Ťת��̬
    ay=-1*pi/2;                                         %%ȡy��岹�Ƕ�
    az=0;                                         %%ȡz��岹�Ƕ�
    px=x(i);py=y(i);pz=z(i);
    Theta1(i)=atan2(-py,-px);
    R(:,:,i)=[cos(Theta1(i)) -sin(Theta1(i)) 0;sin(Theta1(i)) cos(Theta1(i)) 0; 0 0 1]*[cos((i-1)*ay/n) 0 sin((i-1)*ay/n); 0 1 0; -sin((i-1)*ay/n) 0 cos((i-1)*ay/n)];   %�󵥲�·����ÿ��·�������ת����
    r11=R(1,1,i);r12=R(1,2,i);r13=R(1,3,i);r21=R(2,1,i);r22=R(2,2,i);r23=R(2,3,i);r31=R(3,1,i);r32=R(3,2,i);r33=R(3,3,i);
    [a1,a2,a3,a4,a5]=invkinematics_around(r11,r12,r13,px,r21,r22,r23,py,r31,r32,r33,pz);           %��·����ÿһ��������˶�ѧ���õ��Ĺؽڽ�
    Theta1(i)=a1; Theta2(i)=a2; Theta3(i)=a3; Theta4(i)=a4; Theta5(i)=a5;
    if abs(abs(Theta1(i))-pi)<=10^(-5)
        if Theta1(i-1)>0
            Theta1(i)=pi;
        end
        if Theta1(i-1)<0
            Theta1(i)=-pi;
        end
    end
 %%%%%%%%%%%%%%%  �岹��̬  5������� ��ת��̬ 
%  ay=-3*pi/2;                                         %%ȡy��岹�Ƕ�
%     az=0;                                         %%ȡz��岹�Ƕ�
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
% %%%%%%%%%%%%%%%%%%%%%%%����·�����ٶ�(���ڸ����м��ٶȵķ�����
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
%%%%%%%%%%%%%%%%%%%%%%%����n-1��·������ٶȡ��ٶȣ����ڼ��ٶ������ķ�����
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
%%%%%%%%%%%%%%%%%%%%%%%%%%��ÿһ��·���е�λ�á��ٶȡ����ٶ�
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
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%����·���е����� ţ��ŷ������
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
% 
% %%%%%%%%%%%%%%%%%���������ĵ�Чʽ
% ThermalEnergy=0;
% for i=1:n
% J=tor1(i)^2*td(i)+tor2(i)^2*td(i)+tor3(i)^2*td(i)+tor4(i)^2*td(i)+tor5(i)^2*td(i);
% ThermalEnergy=J+ThermalEnergy;
% end




view(3);                                           %%������任�Ƿ���ȷ
grid on;
hold on;
x=[x];y=[y];z=[z];
% line([ps(3),pg(3)],[ps(2),pg(2)],[ps(1),pg(1)]);
plot3(z,y,x,'r*-');
drawpole;
drawrobotjoint;
   