%% ==========================================================
% 函数功能：根据圆柱轴心线PA和PB两点,以及给定的半径画圆柱
% 输入： PA - 轴心第一点坐标向量
%        PB - 轴心第二点坐标向量
%        r - 圆柱半径
%       clr - 圆柱的颜色
%       alpha - 圆柱面的透明度
%       sline - 是否显示两底面边线，0表示不显示
% 输出：hcy - 圆柱3个面的句柄
function hcy = drawcylinder(PA,PB,r,clr,alpha,sline)
%求出轴心线长，即为圆柱高度
PAB = PB-PA;
cylen = norm(PAB);

%生成Z轴上的基本圆柱
Ndiv = 20;
%侧面
[a,zs]=ndgrid((0:.05:1)*2*pi,0:cylen/Ndiv:cylen);
xs=cos(a)*r;
ys=sin(a)*r;
%底面
[a,rb]=ndgrid((0:.05:1)*2*pi,[0 r]);
xb=cos(a).*rb;
yb=sin(a).*rb;
zb=xb*0;
%顶面
[a,ru]=ndgrid((0:.05:1)*2*pi,[0 r]);
xu=cos(a).*ru;
yu=sin(a).*ru;
zu=xu*0 + cylen;

%Z轴上的杆件
PZ = [0,0,cylen];
%向量夹角
theta = acos(dot(PAB,PZ)/(norm(PAB)*norm(PZ)));

if (abs(theta - pi) < 1e-3)    %theta在pi附近，反向
    
    RotK = [1 0 0; 0 -1 0; 0 0 -1];
    
elseif (theta < 1e-3)  %theta在0附近，同向
    
    RotK = [1 0 0; 0 1 0; 0 0 1]; 

else                 %其他情况
    
    %向量叉积，表示旋转轴,注意求的是从PZ到PAB的！！
    Cross_P = cross(PZ,PAB);
    K = Cross_P/norm(Cross_P);
    
    %求旋转矩阵
    St = sin(theta);
    Ct = cos(theta);
    Vt = 1 - Ct;
    KxxVt = K(1)^2*Vt;
    KyyVt = K(2)^2*Vt;
    KzzVt = K(3)^2*Vt;
    KxyVt = K(1)*K(2)*Vt;
    KxzVt = K(1)*K(3)*Vt;
    KyzVt = K(2)*K(3)*Vt;
    RotK = [KxxVt+Ct KxyVt-K(3)*St KxzVt+K(2)*St;
        KxyVt+K(3)*St KyyVt+Ct KyzVt-K(1)*St;
        KxzVt-K(2)*St KyzVt+K(1)*St KzzVt+Ct];
    
end

    %对圆柱进行变换
    [M,N] = size(xs);
    for i = 1:M
        for j = 1:N
            PRoted = RotK*[xs(i,j);ys(i,j);zs(i,j)];
            xs(i,j) = PRoted(1) + PA(1);
            ys(i,j) = PRoted(2) + PA(2);
            zs(i,j) = PRoted(3) + PA(3);
        end
    end
    
    [M,N] = size(xb);
    for i = 1:M
        for j = 1:N
            PRoted = RotK*[xb(i,j);yb(i,j);zb(i,j)];
            xb(i,j) = PRoted(1) + PA(1);
            yb(i,j) = PRoted(2) + PA(2);
            zb(i,j) = PRoted(3) + PA(3);
        end
    end
    
    [M,N] = size(xu);
    for i = 1:M
        for j = 1:N
            PRoted = RotK*[xu(i,j);yu(i,j);zu(i,j)];
            xu(i,j) = PRoted(1) + PA(1);
            yu(i,j) = PRoted(2) + PA(2);
            zu(i,j) = PRoted(3) + PA(3);
        end
    end

h1 = surf(xs,ys,zs,'EdgeColor','none','Facecolor',clr,'FaceAlpha',alpha);
hold on;
h2 = surf(xb,yb,zb,'EdgeColor','none','Facecolor',clr,'FaceAlpha',alpha);
hold on;
h3 = surf(xu,yu,zu,'EdgeColor','none','Facecolor',clr,'FaceAlpha',alpha);
hold on;

if sline
    plot3(xb(:,2),yb(:,2),zb(:,2),'k','linewidth',1);
    hold on;
    plot3(xu(:,2),yu(:,2),zu(:,2),'k','linewidth',1);
    hold on;
end

hcy = [h1 h2 h3];

end