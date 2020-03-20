%% ==========================================================
% �������ܣ���������
% ���룺 PA,PB - ������������ߵ������˵�
%       Pr - �������ߴ�ֱ����ĶԽ��߳�
%       theta - �����������ߵ�ת��
%       clr - ��ɫ
%
function drawcube(PA,PB,Pr,theta,clr,faceA)

if nargin < 6
    faceA = 1;
end
% ������ͼ��ʱ����������⣬Ӧ�ü���45��
theta = theta + 45;

%���ͱ�׼���˼�����
PAB = PB - PA;
AB_Len = norm(PAB);
PAB = PAB/AB_Len;

% ��ֹ������С������atan2���ʱ�����Ƚϴ��ת��
for i = 1:length(PAB)
    if abs(PAB(i)) < 1e-12
        PAB(i) = 0;
    end
end

%�������������Alpha��Beta
Alpha = atan2(PAB(2),PAB(1)); %��Z����ת
Beta = asin(-PAB(3));      %ע�⣺��Y����ת�Ľ������X�����Ÿ˼����򣡣�����������������ŷ�������廹��һ��
theta = deg2rad(theta);

CA = cos(Alpha); SA = sin(Alpha);
CB = cos(Beta); SB = sin(Beta);
CR = cos(theta); SR = sin(theta);

RMatrix = [CA*CB CA*SB*SR-SA*CR CA*SB*CR+SA*SR;
           SA*CB SA*SB*SR+CA*CR SA*SB*CR-CA*SR;
            -SB   CB*SR          CB*CR];
        
%������X��������ķ��˵ĵ�
b = sqrt(2)*Pr/2; % b=a/2
Points = ones(8,3);
Points(1,:) = [0;b;b];
Points(2,:) = [0;b;-b];
Points(3,:) = [0;-b;-b];
Points(4,:) = [0;-b;b];
Points(5,:) = [AB_Len;b;b];
Points(6,:) = [AB_Len;b;-b];
Points(7,:) = [AB_Len;-b;-b];
Points(8,:) = [AB_Len;-b;b];

%ÿ�����Ӧ�����������
VIndex = [1:4; 5:8; 1,2,6,5; 2,3,7,6; 3,4,8,7;1,5,8,4];

%��ת����
for i = 1:8
    Points(i,:) = RMatrix * [Points(i,1);Points(i,2);Points(i,3)] + [PA(1);PA(2);PA(3)];
end


for i=1:6
    patch(Points(VIndex(i,:),1),Points(VIndex(i,:),2),Points(VIndex(i,:),3),clr,'FaceAlpha',faceA);
    hold on;
end

end