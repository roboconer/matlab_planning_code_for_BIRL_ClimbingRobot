function hRobot = DrawRobotmo(robot,transparency,mode)
%% ����
% DrawRobot��һ��ר���ڻ�������������Climbot-5D����Climbot-6D�ĺ����������˵ļг�λ�ú͹������������뷽ʽ��
%       ��ʽһ��ͨ��CurGraspMtx��TarJAs���ֱ�ָ��ȫ������ϵ�µ�ǰ�г�λ�ˣ���ξ��󣩺͹ؽڽ�������
%       ��ʽ����ͨ��CurGraspMtx��TarGraspMtx���ֱ����ȫ������ϵ�µĵ�ǰ�г�λ�˾���;ֲ�����ϵ�����Ե�ǰ�г�����Ϊ�ο����µ�Ŀ��г�λ�˾���
% ����: robot - �ṹ�壬�����¶�ѡһ��
%           ע�⣡����Ϊ�������˵���λ����ɹ��ͣ���ͼʱĬ��robot.TarJAs - [0,90,0,90,0]
%           'TarJAs',[0,0,0,0,0]��G1���£�G2����ʱ�ĹؽڽǶ�ֵ��
%           'TarJAs',[0,180,0,180,0] G1���ϣ�G2����ʱ�ĹؽڽǶ�ֵ
%       Climbot5D = struct('DoFs',5,'LinksLen',[436.3,293.2,293.2,436.3],'ModuleRadii',50,...
%           'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%           'TarJAs',[],'hIKine',@IKine5D,'hLink',@Linkage5D);
%           ע�⣡����Ϊ�������˵���λ����ɹ��ͣ���ͼʱĬ��robot.TarJAs - [0,90,90,0,0,0]
%           'TarJAs',[0,0,0,0,0] G1���£�G2����ʱ�ĹؽڽǶ�ֵ
%           'TarJAs',[0,180,0,180,0]] G1���ϣ�G2����ʱ�ĹؽڽǶ�ֵ
%       Climbot6D = struct('DoFs',6,'LinksLen',[436.3,293.2,213.1,190.3,436.3],'ModuleRadii',50,...
%           'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,6),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%           'TarJAs',[0,0,90,0,-90,0],'hIKine',@IKine6D,'hLink',@Linkage6D); 
%       transparency - ͸�������ã�ȡֵ[0,1]����ֵԽС͸����Խ�ߣ�
%       mode - ��������ģʽ�Ĳ������ǿ�ֵ������ʹ�÷�ʽһ��������л�ͼ��ȱʡ���߿�ֵ��Ӧ��ʽ����
% �����hRobot - ���Ƶ�ͼ��Ԫ����Ӧ�ľ������ΪMoveRobot()������������ƶ�ͼ��Ԫ���Ӷ������˶�Ч����

% ��ע����Ӧ���������˳���
% l0 = 282.12; l1 = 107; l2 = l3 = 375; l4 = l1; l5 = l0;

% ============= �������뷽ʽȷ���Ƿ���Ҫ����� ===============
% if (nargin < 3) || isempty(mode)
%     % �������ǻ����˵�ĩ��λ�ˣ����û����˵���Ϣ����������
%     [ikflag,robot.TarJAs] = robot.hIKine(robot);
%     if ~ikflag
%         disp('û����⣬DrawRobotʵ��û����...');
%         hRobot = [];
%         return;
%     end
% end

% ========== �����������λ��ؽ���λ��ͳһ������ ==========
if robot.DoFs == 5
    JAs = robot.TarJAs - [0,90,0,90,0];
elseif robot.DoFs == 6
    JAs = robot.TarJAs - [0,90,90,0,0,0];
else
    error('DrawRobotֻ�ܻ�5���ɶȻ�6���ɶȵ�����������');
end

hRobot = zeros(1,24);   % ��������˸����㲿���ľ����������ʱ��
% ======== �������гֶ˵����� ==========
 if robot.FixedGripper == 2
%     % ԭ����Ŀ������ϵ�������ڵĻ�����ϵ����������ϵ��
%     robot.CurGraspMtx = robot.CurGraspMtx * robot.TarGraspMtx;
%     % �����гֶ�ʱ�����ݽ�����ϵ�Ĺ���y��z�ᷴת
%     robot.CurGraspMtx(1:3,2:3) = [-robot.CurGraspMtx(1:3,2),-robot.CurGraspMtx(1:3,3)];
DrawCor = robot.CurGraspMtx*tl(0,0,85);   % ��ͼ����
% ��һ���г���

hRobot(1) = drawS(DrawCor,transparency);
% ����
hRobot(2) = drawBC(DrawCor,transparency);

% �ƶ���Iģ���ĩ��
DrawCor = DrawCor*tl(0,0,163.4);
% ��һ��Iģ��
hRobot(3) = drawI(DrawCor,transparency);

% ����
hRobot(4) = drawBC(DrawCor,transparency);


    
% �ƶ�����һ��Tģ��͵ڶ���Tģ��Ľ�ϴ�
DrawCor = DrawCor*rz(JAs(1)+180)*tl(0,0,93.4)*ry(JAs(2))*tl(0,0,146.6);
% ��һ��Tģ��
hRobot(5:6) = drawT(DrawCor*ry(180),-JAs(2),transparency);
% ����
hRobot(7) = drawBC(DrawCor,transparency);
 JANum = 4;
    % �ӳ���Ͳ
    hRobot(11) = drawH(DrawCor,transparency);
    
    % �ƶ�����Ͳĩ��
    DrawCor = DrawCor*tl(0,0,53.2);
    %����
   hRobot(10) = drawBC(DrawCor,transparency); 
   % �ƶ����ڶ���Tģ���ĩ��
DrawCor = DrawCor*tl(0,0,93.4)*ry(JAs(3))*tl(0,0,146.6);
% �ڶ���Tģ��
hRobot(8:9) = drawT(DrawCor*ry(180),-JAs(3),transparency);




%---- 5/6���ɶȲ�ͬ�Ĳ��� ----%
% if robot.DoFs == 6
%     DrawCor = DrawCor*tl(0,0,163.4)*rz(JAs(4));
%     hRobot(11) = drawI(DrawCor,transparency);
%     JANum = 5;
% else  % 5���ɶȵ�
%     JANum = 4;
%     % �ӳ���Ͳ
%     hRobot(11) = drawH(DrawCor,transparency);
%     % �ƶ�����Ͳĩ��
%     DrawCor = DrawCor*tl(0,0,53.2);
% end

% ����
hRobot(12) = drawBC(DrawCor,transparency);

% ������Tģ��
hRobot(13:14) = drawT(DrawCor,JAs(JANum),transparency);

% �ƶ���������Tģ��ĩ��
DrawCor = DrawCor*tl(0,0,146.6)*ry(JAs(JANum))*tl(0,0,93.4);
% ����
hRobot(15) = drawBC(DrawCor,transparency);

% �ƶ����ڶ���Iģ��ĩ��
DrawCor = DrawCor*tl(0,0,163.4);
% Iģ��
hRobot(16) = drawI(DrawCor,transparency);
% ����
hRobot(17) = drawBC(DrawCor,transparency);
% �ڶ����г���

hRobot(18) = drawS(DrawCor*rz(JAs(JANum+1)+180)*ry(180),transparency);
 end

% ======== ���¿�ʼ��ͼ�� ==========



% ��5/6DoFs��������˵���ڶ���T�ؽ�֮ǰ�Ĳ��ֶ���һ����
if robot.FixedGripper == 1
% ���ݵ�ǰ�гֵ�Լг����Ķ�����б任�������ƶ����г�����ԭ��
DrawCor = robot.CurGraspMtx*tl(0,0,85);   % ��ͼ����
% ��һ���г���
%hRobot(1) = drawG(DrawCor,transparency);
hRobot(1) = drawS(DrawCor,transparency);
% ����
hRobot(2) = drawBC(DrawCor,transparency);

% �ƶ���Iģ���ĩ��
DrawCor = DrawCor*tl(0,0,163.4);
% ��һ��Iģ��
hRobot(3) = drawI(DrawCor,transparency);

% ����
hRobot(4) = drawBC(DrawCor,transparency);

% �ƶ�����һ��Tģ��͵ڶ���Tģ��Ľ�ϴ�
DrawCor = DrawCor*rz(JAs(1))*tl(0,0,93.4)*ry(-JAs(2))*tl(0,0,146.6);
% ��һ��Tģ��
hRobot(5:6) = drawT(DrawCor*ry(180),JAs(2),transparency);
% ����
hRobot(7) = drawBC(DrawCor,transparency);
% �ڶ���Tģ��
hRobot(8:9) = drawT(DrawCor,-JAs(3),transparency);

% �ƶ����ڶ���Tģ���ĩ��
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(3))*tl(0,0,93.4);
% ����
hRobot(10) = drawBC(DrawCor,transparency);

%---- 5/6���ɶȲ�ͬ�Ĳ��� ----%
if robot.DoFs == 6
    DrawCor = DrawCor*tl(0,0,163.4)*rz(JAs(4));
    hRobot(11) = drawI(DrawCor,transparency);
    JANum = 5;
else  % 5���ɶȵ�
    JANum = 4;
    % �ӳ���Ͳ
    hRobot(11) = drawH(DrawCor,transparency);
    % �ƶ�����Ͳĩ��
    DrawCor = DrawCor*tl(0,0,53.2);
end

% ����
hRobot(12) = drawBC(DrawCor,transparency);

% ������Tģ��
hRobot(13:14) = drawT(DrawCor,-JAs(JANum),transparency);

% �ƶ���������Tģ��ĩ��
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(JANum))*tl(0,0,93.4);
% ����
hRobot(15) = drawBC(DrawCor,transparency);

% �ƶ����ڶ���Iģ��ĩ��
DrawCor = DrawCor*tl(0,0,163.4);
% Iģ��
hRobot(16) = drawI(DrawCor,transparency);
% ����
hRobot(17) = drawBC(DrawCor,transparency);
% �ڶ����г���
%hRobot(18) = drawG(DrawCor*rz(JAs(JANum+1))*ry(180),transparency);
hRobot(18) = drawS(DrawCor*rz(JAs(JANum+1))*ry(180),transparency);
end
end

function hG = drawG(T,FaceA)

global GModule;

V = [GModule.vertices'; ones(1,length(GModule.vertices))];
V = T*V;
hG = patch('faces', GModule.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', GModule.facec, 'EdgeColor','none', 'FaceAlpha',FaceA);

end

function hS = drawS(T,FaceA)
 global SModule;
V = [SModule.vertices'; ones(1,length(SModule.vertices))];
V = T*V;
hS = patch('faces', SModule.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', SModule.facec, 'EdgeColor', 'none', 'FaceAlpha' , FaceA);
end

function hB = drawBC(T,FaceA)

global BandClamp;

V = [BandClamp.vertices'; ones(1,length(BandClamp.vertices))];
V = T*V;
hB = patch('faces', BandClamp.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', BandClamp.facec, 'EdgeColor','none', 'FaceAlpha',FaceA);

end

function hI = drawI(T,FaceA)

global IModule; 

V = [IModule.vertices'; ones(1,length(IModule.vertices))];
V = T*V;
hI = patch('faces', IModule.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', IModule.facec, 'EdgeColor','none', 'FaceAlpha',FaceA);

end

function hT = drawT(T,Angle,FaceA)

global TModuleP1 TModuleP2;

V = [TModuleP1.vertices'; ones(1,length(TModuleP1.vertices))];
V = T*V;
hT1 = patch('faces', TModuleP1.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', TModuleP1.facec, 'EdgeColor','none', 'FaceAlpha',FaceA);

%T = T*ry(Angle);
V = [TModuleP2.vertices'; ones(1,length(TModuleP2.vertices))];
V = T*tl(0,0,146.6)*ry(Angle)*V;
hT2 = patch('faces', TModuleP2.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', TModuleP2.facec, 'EdgeColor','none', 'FaceAlpha',FaceA);

hT = [hT1,hT2];

end

function hH = drawH(T,FaceA)

global HollowPart;

V = HollowPart.vertices';
V = [V(1,:); V(2,:); V(3,:); ones(1,length(V))];
V = T*V;
hH = patch('faces', HollowPart.faces, 'vertices', V(1:3,:)', 'facec', 'flat', 'facec', HollowPart.facec, 'EdgeColor','none', 'FaceAlpha',FaceA);

end

% Homogeneous manipulation functions follow:
%
function Rx = rx(THETA)
% ROTATION ABOUT THE X-AXIS
%
% Rx = rx(THETA)
%
% This is the homogeneous transformation for
% rotation about the X-axis.
%
%	    NOTE:  The angle THETA must be in DEGREES.
%
THETA = THETA*pi/180;  % Note: THETA in radians.
c = cos(THETA);
s = sin(THETA);
Rx = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end
%
function Ry = ry(THETA)
% ROTATION ABOUT THE Y-AXIS
%
% Ry = ry(THETA)
%
% This is the homogeneous transformation for
% rotation about the Y-axis.
%
%		NOTE: The angel THETA must be in DEGREES.
%
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Ry = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
%
function Rz = rz(THETA)
% ROTATION ABOUT THE Z-AXIS
%
% Rz = rz(THETA)
%
% This is the homogeneous transformation for
% rotation about the Z-axis.
%
%		NOTE:  The angle THETA must be in DEGREES.
%
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Rz = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end

function T = tl(x,y,z)
% TRANSLATION ALONG THE X, Y, AND Z AXES
%
% T = tl(x,y,z)
%
% This is the homogeneous transformation for
% translation along the X, Y, and Z axes.
%
T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
end