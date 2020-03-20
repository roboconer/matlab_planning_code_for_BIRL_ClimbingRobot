function [ P ] = Bspline3(pt)
%pt=load('data.txt');
m=length(pt);
n=m+1;
k=3;
U=zeros(1,n+k+2);%节点矢量
%********节点矢量************%
x=pt(:,1);
y=pt(:,2);
z=pt(:,3);
temp=zeros(1,m-1);
for i=1:m-1%m个数据点，利用积累弦长法
temp(i)=sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2+(z(i+1)-z(i))^2);
end
for i=1:k+1%前k+1个节点为0
    U(i)=0;
end
for i=n+k-1:n+k+2%后k+1个节点为1
    U(i)=1;
end
for i=k+1:m+k-1%m个数据点，m=n-1,内节点为m-2个，U(k+1)作为初始值
    U(i+1)=U(i)+temp(i-k);%m-1段弦长，U（k+1）到U（m+k）共m个节点
end
for i=k+1:m+k
    U(i)=U(i)/U(m+k);
end
%***************反算n+1个控制点**************
dU=zeros(1,n+k+2);%△U
dpt1=[-0.6489  0.7079  0.2789];%给定首数据点切矢
dptm=[0.1227  0.7740 -0.6212];%给定末数据点切矢
A=zeros(n-1);
E=zeros(n-1,3);
for i=k+1:n+k+1
    dU(i)=U(i+1)-U(i);
end
A(1,1)=1;%切矢条件b1=1,c1=a1=0,e1
E(1,:)=pt(1,:)+(dU(4)/3)*dpt1;
A(n-1,n-1)=1;%切矢条件cn-1=an-1=0,bn-1=0
E(n-1,:)=pt(m,:)-(dU(n+1)/3)*dptm;
for i=2:n-2%a,b,c,e的值
    A(i,i-1)=dU(i+3).^2/(dU(i+1)+dU(i+2)+dU(i+3));
    A(i,i)=dU(i+3)*(dU(i+1)+dU(i+2))/(dU(i+1)+dU(i+2)+dU(i+3))+...
        dU(i+2)*(dU(i+3)+dU(i+4))/(dU(i+2)+dU(i+3)+dU(i+4));
    A(i,i+1)=dU(i+2).^2/(dU(i+2)+dU(i+3)+dU(i+4));
   E(i,:)=(dU(i+2)+dU(i+3))*pt(i,:);
end
D=A\E;
D=[pt(1,:);D;pt(m,:)];%加上首末端点，控制顶点比数据点多两个，D为控制顶点矩阵
[s,t]=size(D);
%矩阵行数由1开头，故除k和l以外的参数都加1,n本身已加1
dt=0.01;
k=3;
P=[];
dP=[];
ddP=[];
syms x;
%定义节点间的递增变量为变量x
u_0(1:k+1)=0;
for t=k+2:s
    u_0(t)=u_0(t-1)+1/(s-k);
end
u_0(s+1:s+k+1)=1;
%求出初始的节点值

%r=0,数据点：
for i=k+1:s
    u=u_0;
    d=sym(D);
    %每次迭代后将d恢复初值
    for l=1:k
        for j=i-k:i-l
            alpha(j)=(x-u(j+l))/(u(j+k+1)-u(j+l));
            d(j,:)=(1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
        end
    end
    %带入变量x求出[ui.ui+1]内l=3的控制点d
    M=subs(d(i-k,:),x,(u(i):dt:(u(i+1)-dt))');
    P=[P;double(M)];
    %带入x的值求得各节点区间内的点
end
M=subs(d(i-k,:),x,1);
P=[P;double(M)];
%补上最后一个点
% plot3(pt(:,1),pt(:,2),pt(:,3),'*r');
% hold on;
% plot3(D(:,1),D(:,2),D(:,3),'b-o');
% hold on;
 %plot3(P(:,1),P(:,2),P(:,3),'k','Linewidth',1);
 hold on;
grid on;
%view(0,90)
%r=1,一阶单位切矢量：
for i=k+1:s
    for j=i-k:i-1
        pt_1(j,:)=k*(D(j+1,:)-D(j,:))/(u_0(j+k+1)-u_0(j+1));
    end
end
u_1=u_0;
u_1(1)=[];
u_1(s+k)=[];
k1=k-1;
s1=s-1;
for i=k1+1:s1
    u=u_1;
    d=sym(pt_1);
    for l=1:k1
        for j=i-k1:i-l
            alpha(j)=(x-u(j+l))/(u(j+k1+1)-u(j+l));
            d(j,:)=(1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
        end
    end
    M=subs(d(i-k1,:),x,(u(i):dt:(u(i+1)-dt))');
    dP=[dP;double(M)];
end
M=subs(d(i-k1,:),x,1);
dP=[dP;double(M)];

%r=2,二阶单位切矢量：
for i=k+1:s
    for j=i-k:i-2
        d_2(j,:)=(k-1)*(pt_1(j+1,:)-pt_1(j,:))/(u_0(j+k+1)-u_0(j+2));
    end
end
u_2=u_1;
u_2(1)=[];
u_2(s+k-2)=[];
k2=k-2;
n2=s-2;
for i=k2+1:n2
    u=u_2;
    d=sym(d_2);
    for l=1:k2
        for j=i-k2:i-l
            alpha(j)=(x-u(j+l))/(u(j+k2+1)-u(j+l));
            d(j,:)=(1-alpha(j))*d(j,:)+alpha(j)*d(j+1,:);
        end
    end
    M=subs(d(i-k2,:),x,(u(i):dt:(u(i+1)-dt))');
    ddP=[ddP;double(M)];
end
M=subs(d(i-k2,:),x,1);
ddP=[ddP;double(M)];
%单位化一阶、二阶切矢量:
% for t=1:length(dP)
%     dP(t,:)=dP(t,:)/norm(dP(t,:));
%     ddP(t,:)=ddP(t,:)/norm(ddP(t,:));
%     quiver3(P(t,1),P(t,2),P(t,3),dP(t,1),dP(t,2),dP(t,3),'k');
%     hold on;
%     quiver3(P(t,1),P(t,2),P(t,3),ddP(t,1),ddP(t,2),ddP(t,3),'g');
%     hold on;
% end
end


