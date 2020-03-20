%% 函数功能：画图辅助函数，标示三维坐标系
% 输入：HMtx - 齐次矩阵，标示三维坐标系应该画的位置和姿态
%      length - 箭头的长度，对于爬杆机器人的场景，设为500较为恰当
%      offsect - 坐标系的偏置，为避免被机器人或者杆件阻挡
%      axisname - 坐标轴的名字
function drawcoordinate(HMtx,length,offsect,axisname)

if isempty(length)
    length = 350*ones(1,3);
end
% 为避免机器人和杆件的遮挡，沿Y轴偏移一段距离
if isempty(offsect)
    offsect = [0 200 0];
end
if isempty(axisname)
    axisname = 'noa';
end

% 坐标系的位置
Position = HMtx(1:3,4);
% invRMtx = inv(HMtx(1:3,1:3));
% offsect = tranbyHMtx(offsect,invRMtx);
offsect = HMtx(1:3,1:3) \ transpose(offsect);
Position = Position + offsect;
Endpoints = diag(length);
Endpoints = tranbyHMtx(Endpoints,HMtx)+repmat(offsect,1,3);

%坐标轴的颜色
clr = 'rgb';
%clr = 'rbk';
textoffsect = 100*HMtx(1:3,1:3) + Endpoints;

for i = 1:3
    mArrow3(transpose(Position),Endpoints(i,:),'stemWidth',12,'color',clr(i)); % 12
    %text(textoffsect(i,1),textoffsect(i,2),textoffsect(i,3),axisname(i),...
        %'FontSize',20,'FontName','Times New Roman','FontAngle','italic');
end

    hold on;
    
end