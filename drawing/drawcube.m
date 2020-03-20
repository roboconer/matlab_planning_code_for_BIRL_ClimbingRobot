%% ==========================================================
% 函数功能：画长方体
% 输入： PA,PB - 长方体的轴心线的两个端点
%       Pr - 与轴心线垂直的面的对角线长
%       theta - 长方体轴心线的转角
%       clr - 颜色
%
function drawcube(PA,PB,Pr,theta,clr,faceA)

if nargin < 6
    faceA = 1;
end
% 当初画图的时候的设置问题，应该加上45°
theta = theta + 45;

%求解和标准化杆件向量
PAB = PB - PA;
AB_Len = norm(PAB);
PAB = PAB/AB_Len;

% 防止两个很小的数用atan2求解时出来比较大的转角
for i = 1:length(PAB)
    if abs(PAB(i)) < 1e-12
        PAB(i) = 0;
    end
end

%根据球坐标求解Alpha和Beta
Alpha = atan2(PAB(2),PAB(1)); %绕Z轴旋转
Beta = asin(-PAB(3));      %注意：绕Y轴旋转的结果是令X轴沿着杆件方向！！！跟纯粹的球坐标或欧拉角意义还不一样
theta = deg2rad(theta);

CA = cos(Alpha); SA = sin(Alpha);
CB = cos(Beta); SB = sin(Beta);
CR = cos(theta); SR = sin(theta);

RMatrix = [CA*CB CA*SB*SR-SA*CR CA*SB*CR+SA*SR;
           SA*CB SA*SB*SR+CA*CR SA*SB*CR-CA*SR;
            -SB   CB*SR          CB*CR];
        
%产生沿X轴正方向的方杆的点
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

%每个面对应顶点的索引号
VIndex = [1:4; 5:8; 1,2,6,5; 2,3,7,6; 3,4,8,7;1,5,8,4];

%旋转顶点
for i = 1:8
    Points(i,:) = RMatrix * [Points(i,1);Points(i,2);Points(i,3)] + [PA(1);PA(2);PA(3)];
end


for i=1:6
    patch(Points(VIndex(i,:),1),Points(VIndex(i,:),2),Points(VIndex(i,:),3),clr,'FaceAlpha',faceA);
    hold on;
end

end