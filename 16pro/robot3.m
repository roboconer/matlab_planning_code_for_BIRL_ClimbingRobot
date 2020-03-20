L(1) = Link([0 0.675 0.26 pi/2 0]);
L(2) = Link([pi/2 0 0.645 pi 0]);
L(3) = Link([0 0 0.035 -pi/2 0]);
L(4) = Link([0 0.67 0 pi/2 0]);
L(5) = Link([pi 0 0 pi/2 0]);
L(6) = Link([0 0.115 0 0 0]);
r6 =SerialLink(L);%机械臂
r6.name  = '6R机械臂';
q1 = [0 pi/2 0 0 pi 0];
q2 = [pi/4 pi/2 pi/8 pi/2 7*pi/6 pi/2];
t = [0:0.05:4.5];
[q,qd,qdd] = jtraj(q1,q2,t);
plot(r6,q);
figure;
%节点角度变化图
plot(t,q(:,1),t,q(:,2),'--',t,q(:,3),'.',t,q(:,4),'+',t,q(:,5),'-.',t,q(:,6),'*');
legend('节点1','节点2','节点3','节点4','节点5','节点6');
title('各关节角度变化');
xlabel('时间（s）');
ylabel('关节角度（rad）');
grid on;
%节点角速度变化图
figure;
plot(t,qd(:,1),t,qd(:,2),'--',t,qd(:,3),'.',t,qd(:,4),'+',t,qd(:,5),'-.',t,qd(:,6),'*');
legend('节点1','节点2','节点3','节点4','节点5','节点6');
title('各关节角速度变化');
xlabel('时间（s）');
ylabel('关节角速度（rad/s）');
grid on;
%节点角加度变化图
figure;
plot(t,qdd(:,1),t,qdd(:,2),'--',t,qdd(:,3),'.',t,qdd(:,4),'+',t,qdd(:,5),'-.',t,qdd(:,6),'*');
legend('节点1','节点2','节点3','节点4','节点5','节点6');
title('各关节角加速度变化');
grid on
xlabel('时间（s）');
ylabel('关节角速度（rad/s^2）');
%运动轨迹
T=fkine(r6,q);
x(1,1:91)=T(1,4,:);
y(1,1:91)=T(2,4,:);
z(1,1:91)=T(3,4,:);
figure;
plot3(x,y,z,'k');
grid on;
