function ColorPole(pole,GR,interval,paraVelAniso,ptNum)

Dir = (pole(4:6)-pole(1:3))/norm(pole(4:6)-pole(1:3));

% �ֶ��� -- ����GR(i)��GR(i+1)Ϊһ�οɼг�������յ�
segNum = length(GR)/2;

count = 1;

for i = 1:segNum

ColorCylinder(pole(1:3),Dir,GR(2*i-1),GR(2*i),interval,paraVelAniso(count:count+ptNum(i)-1));

count = count + ptNum(i);

end

% colormapping([0; paraVelAniso; 1]);
colormap(autumn(1024));
% colorbar;
freezeColors;

colormap('jet');

end

function hcy = ColorCylinder(RefPt,Dir,t_beg,t_end,interval,colorIndex)

%��������߳�����ΪԲ���߶�
cylen = abs(t_end-t_beg);

%Բ����Ĭ�Ϸ�20��
Rdiv = 20;
% �˼���ʵ�뾶30
r = 32;

% ����Z���ϵ�Բ�ܵĲ���
[a,zs]=ndgrid((0:1/Rdiv:1)*2*pi,0:interval:cylen);
xs=cos(a)*r;
ys=sin(a)*r;

%Z���ϵĸ˼�
PZ = [0,0,1];
%�����н�
theta = acos(dot(Dir,PZ));

if (abs(theta - pi) < 1e-3)    %theta��pi����������
    
    RotK = [1 0 0; 0 -1 0; 0 0 -1];
    
elseif (theta < 1e-3)  %theta��0������ͬ��
    
    RotK = [1 0 0; 0 1 0; 0 0 1];
    
else                 %�������
    
    %�����������ʾ��ת��,ע������Ǵ�PZ��PAB�ģ���
    Cross_P = cross(PZ,Dir);
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
P0 = RefPt + t_beg * Dir;
[M N] = size(xs);
for i = 1:M
    for j = 1:N
        PRoted = RotK*[xs(i,j);ys(i,j);zs(i,j)];
        xs(i,j) = PRoted(1) + P0(1);
        ys(i,j) = PRoted(2) + P0(2);
        zs(i,j) = PRoted(3) + P0(3);
    end
end

% colorIndex

clr = repmat(colorIndex,Rdiv+1,1);

%һ���Ի���Բ���Ĳ���
hcy = surf(xs,ys,zs,clr,'EdgeColor','none');
hold on;

end

function colormapping(u)

newmap = colormap(autumn(length(u(:))));

colormap(newmap);

%cbfreeze(newmap)
% newmap = maptem(round(linspace(0, size(maptem, 1), ind)), :);

end