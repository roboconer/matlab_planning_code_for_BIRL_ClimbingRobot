%% �������ܣ���ͼ������������ʾ��ά����ϵ
% ���룺HMtx - ��ξ��󣬱�ʾ��ά����ϵӦ�û���λ�ú���̬
%      length - ��ͷ�ĳ��ȣ��������˻����˵ĳ�������Ϊ500��Ϊǡ��
%      offsect - ����ϵ��ƫ�ã�Ϊ���ⱻ�����˻��߸˼��赲
%      axisname - �����������
function drawcoordinate(HMtx,length,offsect,axisname)

if isempty(length)
    length = 350*ones(1,3);
end
% Ϊ��������˺͸˼����ڵ�����Y��ƫ��һ�ξ���
if isempty(offsect)
    offsect = [0 200 0];
end
if isempty(axisname)
    axisname = 'noa';
end

% ����ϵ��λ��
Position = HMtx(1:3,4);
% invRMtx = inv(HMtx(1:3,1:3));
% offsect = tranbyHMtx(offsect,invRMtx);
offsect = HMtx(1:3,1:3) \ transpose(offsect);
Position = Position + offsect;
Endpoints = diag(length);
Endpoints = tranbyHMtx(Endpoints,HMtx)+repmat(offsect,1,3);

%���������ɫ
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