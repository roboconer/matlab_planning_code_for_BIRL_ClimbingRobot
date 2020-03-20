function hRobot = DrawRobotmo(robot,transparency,mode)
%% 描述
% DrawRobot是一个专用于绘制攀爬机器人Climbot-5D或者Climbot-6D的函数。机器人的夹持位置和构型有两种输入方式：
%       方式一：通过CurGraspMtx和TarJAs来分别指定全局坐标系下当前夹持位姿（齐次矩阵）和关节角向量；
%       方式二：通过CurGraspMtx和TarGraspMtx来分别给定全局坐标系下的当前夹持位姿矩阵和局部坐标系（即以当前夹持器作为参考）下的目标夹持位姿矩阵。
% 输入: robot - 结构体，从以下二选一：
%           注意！！！为将机器人的零位定义成弓型，画图时默认robot.TarJAs - [0,90,0,90,0]
%           'TarJAs',[0,0,0,0,0]，G1在下，G2在上时的关节角度值；
%           'TarJAs',[0,180,0,180,0] G1在上，G2在下时的关节角度值
%       Climbot5D = struct('DoFs',5,'LinksLen',[436.3,293.2,293.2,436.3],'ModuleRadii',50,...
%           'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%           'TarJAs',[],'hIKine',@IKine5D,'hLink',@Linkage5D);
%           注意！！！为将机器人的零位定义成弓型，画图时默认robot.TarJAs - [0,90,90,0,0,0]
%           'TarJAs',[0,0,0,0,0] G1在下，G2在上时的关节角度值
%           'TarJAs',[0,180,0,180,0]] G1在上，G2在下时的关节角度值
%       Climbot6D = struct('DoFs',6,'LinksLen',[436.3,293.2,213.1,190.3,436.3],'ModuleRadii',50,...
%           'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,6),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%           'TarJAs',[0,0,90,0,-90,0],'hIKine',@IKine6D,'hLink',@Linkage6D); 
%       transparency - 透明度设置，取值[0,1]，数值越小透明度越高；
%       mode - 控制输入模式的参数，非空值均代表使用方式一的输入进行绘图，缺省或者空值对应方式二。
% 输出：hRobot - 绘制的图单元所对应的句柄，作为MoveRobot()函数输入可以移动图单元，从而制作运动效果。

% 备注：对应机器人连杆长度
% l0 = 282.12; l1 = 107; l2 = l3 = 375; l4 = l1; l5 = l0;

% ============= 按照输入方式确定是否需要求逆解 ===============
% if (nargin < 3) || isempty(mode)
%     % 给定的是机器人的末端位姿，先用机器人的信息把逆解求出来
%     [ikflag,robot.TarJAs] = robot.hIKine(robot);
%     if ~ikflag
%         disp('没有逆解，DrawRobot实在没法画...');
%         hRobot = [];
%         return;
%     end
% end

% ========== 处理机器人零位与关节零位不统一的问题 ==========
if robot.DoFs == 5
    JAs = robot.TarJAs - [0,90,0,90,0];
elseif robot.DoFs == 6
    JAs = robot.TarJAs - [0,90,90,0,0,0];
else
    error('DrawRobot只能画5自由度或6自由度的攀爬机器人');
end

hRobot = zeros(1,24);   % 保存机器人各个零部件的句柄，做动画时用
% ======== 处理交换夹持端的问题 ==========
 if robot.FixedGripper == 2
%     % 原来的目标坐标系就是现在的基座标系，世界坐标系下
%     robot.CurGraspMtx = robot.CurGraspMtx * robot.TarGraspMtx;
%     % 交换夹持端时，根据建坐标系的规则，y和z轴反转
%     robot.CurGraspMtx(1:3,2:3) = [-robot.CurGraspMtx(1:3,2),-robot.CurGraspMtx(1:3,3)];
DrawCor = robot.CurGraspMtx*tl(0,0,85);   % 绘图坐标
% 第一个夹持器

hRobot(1) = drawS(DrawCor,transparency);
% 卡环
hRobot(2) = drawBC(DrawCor,transparency);

% 移动到I模块的末端
DrawCor = DrawCor*tl(0,0,163.4);
% 第一个I模块
hRobot(3) = drawI(DrawCor,transparency);

% 卡环
hRobot(4) = drawBC(DrawCor,transparency);


    
% 移动到第一个T模块和第二个T模块的结合处
DrawCor = DrawCor*rz(JAs(1)+180)*tl(0,0,93.4)*ry(JAs(2))*tl(0,0,146.6);
% 第一个T模块
hRobot(5:6) = drawT(DrawCor*ry(180),-JAs(2),transparency);
% 卡环
hRobot(7) = drawBC(DrawCor,transparency);
 JANum = 4;
    % 加长套筒
    hRobot(11) = drawH(DrawCor,transparency);
    
    % 移动到套筒末端
    DrawCor = DrawCor*tl(0,0,53.2);
    %卡环
   hRobot(10) = drawBC(DrawCor,transparency); 
   % 移动到第二个T模块的末端
DrawCor = DrawCor*tl(0,0,93.4)*ry(JAs(3))*tl(0,0,146.6);
% 第二个T模块
hRobot(8:9) = drawT(DrawCor*ry(180),-JAs(3),transparency);




%---- 5/6自由度不同的部分 ----%
% if robot.DoFs == 6
%     DrawCor = DrawCor*tl(0,0,163.4)*rz(JAs(4));
%     hRobot(11) = drawI(DrawCor,transparency);
%     JANum = 5;
% else  % 5自由度的
%     JANum = 4;
%     % 加长套筒
%     hRobot(11) = drawH(DrawCor,transparency);
%     % 移动到套筒末端
%     DrawCor = DrawCor*tl(0,0,53.2);
% end

% 卡环
hRobot(12) = drawBC(DrawCor,transparency);

% 第三个T模块
hRobot(13:14) = drawT(DrawCor,JAs(JANum),transparency);

% 移动到第三个T模块末端
DrawCor = DrawCor*tl(0,0,146.6)*ry(JAs(JANum))*tl(0,0,93.4);
% 卡环
hRobot(15) = drawBC(DrawCor,transparency);

% 移动到第二个I模块末端
DrawCor = DrawCor*tl(0,0,163.4);
% I模块
hRobot(16) = drawI(DrawCor,transparency);
% 卡环
hRobot(17) = drawBC(DrawCor,transparency);
% 第二个夹持器

hRobot(18) = drawS(DrawCor*rz(JAs(JANum+1)+180)*ry(180),transparency);
 end

% ======== 以下开始绘图了 ==========



% 对5/6DoFs机器人来说，第二个T关节之前的部分都是一样的
if robot.FixedGripper == 1
% 根据当前夹持点对夹持器的顶点进行变换，并且移动到夹持器的原点
DrawCor = robot.CurGraspMtx*tl(0,0,85);   % 绘图坐标
% 第一个夹持器
%hRobot(1) = drawG(DrawCor,transparency);
hRobot(1) = drawS(DrawCor,transparency);
% 卡环
hRobot(2) = drawBC(DrawCor,transparency);

% 移动到I模块的末端
DrawCor = DrawCor*tl(0,0,163.4);
% 第一个I模块
hRobot(3) = drawI(DrawCor,transparency);

% 卡环
hRobot(4) = drawBC(DrawCor,transparency);

% 移动到第一个T模块和第二个T模块的结合处
DrawCor = DrawCor*rz(JAs(1))*tl(0,0,93.4)*ry(-JAs(2))*tl(0,0,146.6);
% 第一个T模块
hRobot(5:6) = drawT(DrawCor*ry(180),JAs(2),transparency);
% 卡环
hRobot(7) = drawBC(DrawCor,transparency);
% 第二个T模块
hRobot(8:9) = drawT(DrawCor,-JAs(3),transparency);

% 移动到第二个T模块的末端
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(3))*tl(0,0,93.4);
% 卡环
hRobot(10) = drawBC(DrawCor,transparency);

%---- 5/6自由度不同的部分 ----%
if robot.DoFs == 6
    DrawCor = DrawCor*tl(0,0,163.4)*rz(JAs(4));
    hRobot(11) = drawI(DrawCor,transparency);
    JANum = 5;
else  % 5自由度的
    JANum = 4;
    % 加长套筒
    hRobot(11) = drawH(DrawCor,transparency);
    % 移动到套筒末端
    DrawCor = DrawCor*tl(0,0,53.2);
end

% 卡环
hRobot(12) = drawBC(DrawCor,transparency);

% 第三个T模块
hRobot(13:14) = drawT(DrawCor,-JAs(JANum),transparency);

% 移动到第三个T模块末端
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(JANum))*tl(0,0,93.4);
% 卡环
hRobot(15) = drawBC(DrawCor,transparency);

% 移动到第二个I模块末端
DrawCor = DrawCor*tl(0,0,163.4);
% I模块
hRobot(16) = drawI(DrawCor,transparency);
% 卡环
hRobot(17) = drawBC(DrawCor,transparency);
% 第二个夹持器
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