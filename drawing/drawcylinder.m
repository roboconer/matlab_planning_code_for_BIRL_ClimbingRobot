%% ==========================================================
% �������ܣ�����Բ��������PA��PB����,�Լ������İ뾶��Բ��
% ���룺 PA - ���ĵ�һ����������
%        PB - ���ĵڶ�����������
%        r - Բ���뾶
%       clr - Բ������ɫ
%       alpha - Բ�����͸����
%       sline - �Ƿ���ʾ��������ߣ�0��ʾ����ʾ
% �����hcy - Բ��3����ľ��
function hcy = drawcylinder(PA,PB,r,clr,alpha,sline)
%��������߳�����ΪԲ���߶�
PAB = PB-PA;
cylen = norm(PAB);

%����Z���ϵĻ���Բ��
Ndiv = 20;
%����
[a,zs]=ndgrid((0:.05:1)*2*pi,0:cylen/Ndiv:cylen);
xs=cos(a)*r;
ys=sin(a)*r;
%����
[a,rb]=ndgrid((0:.05:1)*2*pi,[0 r]);
xb=cos(a).*rb;
yb=sin(a).*rb;
zb=xb*0;
%����
[a,ru]=ndgrid((0:.05:1)*2*pi,[0 r]);
xu=cos(a).*ru;
yu=sin(a).*ru;
zu=xu*0 + cylen;

%Z���ϵĸ˼�
PZ = [0,0,cylen];
%�����н�
theta = acos(dot(PAB,PZ)/(norm(PAB)*norm(PZ)));

if (abs(theta - pi) < 1e-3)    %theta��pi����������
    
    RotK = [1 0 0; 0 -1 0; 0 0 -1];
    
elseif (theta < 1e-3)  %theta��0������ͬ��
    
    RotK = [1 0 0; 0 1 0; 0 0 1]; 

else                 %�������
    
    %�����������ʾ��ת��,ע������Ǵ�PZ��PAB�ģ���
    Cross_P = cross(PZ,PAB);
    K = Cross_P/norm(Cross_P);
    
    %����ת����
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

    %��Բ�����б任
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