%% ==============================================================
% 使机器人一端夹持另一端运动的函数
% 输入：hR - 机器人各连杆句柄，为1*24，从DrawRobot中获得
%       robot - 机器人结构体
%       mode - 指示是否需要先进行逆解求解的标志位（省略或者为空都进行逆解求解）
% 输出：
function MoveRobot(hR,robot,mode)

% ============= 测试是否要求逆解 ===============
% if (nargin < 3) || isempty(mode)
%     % 给定的是机器人的末端位姿，先用机器人的信息把逆解求出来
%     [ikflag,robot.TarJAs] = robot.hIKine(robot);
%     if ~ikflag
%         disp('没有逆解，我实在没法画...');
%         return;
%     end
% end

% ======== 处理机器人零位与关节零位不统一的问题 ==========
if robot.DoFs == 5
    JAs = robot.TarJAs - [0,90,0,90,0];
elseif robot.DoFs == 6
    JAs = robot.TarJAs - [0,90,90,0,0,0];
else
    error('只能画5自由度或6自由度的攀爬机器人');
end

% ======== 处理交换夹持端的问题 ==========
% if robot.FixedGripper == 2
%     % 原来的目标坐标系就是现在的基座标系，世界坐标系下
%     robot.CurGraspMtx = robot.CurGraspMtx * robot.TarGraspMtx;
%     % 交换夹持端时，根据建坐标系的规则，y和z轴反转
%     robot.CurGraspMtx(1:3,2:3) = [-robot.CurGraspMtx(1:3,2),-robot.CurGraspMtx(1:3,3)];
% end

% ======== 以下开始移动机器人的各个零部件 ==========
global BandClamp HollowPart IModule SModule;

% 移动到夹持器原点准备开始画图
if robot.FixedGripper == 2
    
    DrawCor = robot.CurGraspMtx*tl(0,0,85);
    % 第一个夹持器
    UpdateVs(hR(1),DrawCor,SModule.vertices');
    % 卡环
    UpdateVs(hR(2),DrawCor,BandClamp.vertices');
     % 移动到I模块的末端
    DrawCor = DrawCor*tl(0,0,163.4);
    % 第一个I模块
    UpdateVs(hR(3),DrawCor,IModule.vertices');
    
   
    % 卡环
    UpdateVs(hR(4),DrawCor,BandClamp.vertices');
    
    % 移动到第一个T模块和第二个T模块的结合处
    DrawCor = DrawCor*rz(JAs(1)+180)*tl(0,0,93.4)*ry(JAs(2))*tl(0,0,146.6);
    % 第一个T模块
    UpdateTM(hR(5:6),DrawCor*ry(180),-JAs(2));
    
    % 卡环
    UpdateVs(hR(7),DrawCor,BandClamp.vertices');
    JANum = 4;
     % 加长套筒
    UpdateVs(hR(11),DrawCor,HollowPart.vertices');
    % 移动到套筒末端
    DrawCor = DrawCor*tl(0,0,53.2); 
      %卡环
    UpdateVs(hR(10),DrawCor,BandClamp.vertices');
    % 移动到第二个T模块的末端
    DrawCor = DrawCor*tl(0,0,93.4)*ry(JAs(3))*tl(0,0,146.6);
    
    % 第二个T模块
    UpdateTM(hR(8:9),DrawCor*ry(180),-JAs(3));
    
    % 卡环
    UpdateVs(hR(12),DrawCor,BandClamp.vertices');

    % 第三个T模块
    UpdateTM(hR(13:14),DrawCor,JAs(JANum));

    % 移动到第三个T模块末端
    DrawCor = DrawCor*tl(0,0,146.6)*ry(JAs(JANum))*tl(0,0,93.4);
    % 卡环
    UpdateVs(hR(15),DrawCor,BandClamp.vertices');

    % 移动到第二个I模块末端
    DrawCor = DrawCor*tl(0,0,163.4);
    % I模块
    UpdateVs(hR(16),DrawCor,IModule.vertices');
    % 卡环
    UpdateVs(hR(17),DrawCor,BandClamp.vertices');
    % 第二个夹持器
    UpdateVs(hR(18),DrawCor*rz(JAs(JANum+1)+180)*ry(180),SModule.vertices');

    % 刷新画面
  %  drawnow;
    
end
  
if robot.FixedGripper == 1
DrawCor = robot.CurGraspMtx*tl(0,0,85);   % 绘图坐标
% 第一个夹持器
 UpdateVs(hR(1),DrawCor,SModule.vertices');
% 卡环
UpdateVs(hR(2),DrawCor,BandClamp.vertices');

% 移动到I模块的末端
DrawCor = DrawCor*tl(0,0,163.4);
% 第一个I模块
 UpdateVs(hR(3),DrawCor,IModule.vertices');

% 卡环
UpdateVs(hR(4),DrawCor,BandClamp.vertices');

% 移动到第一个T模块和第二个T模块的结合处
DrawCor = DrawCor*rz(JAs(1))*tl(0,0,93.4)*ry(-JAs(2))*tl(0,0,146.6);

% 第一个T模块
UpdateTM(hR(5:6),DrawCor*ry(180),JAs(2));

% 卡环
UpdateVs(hR(7),DrawCor,BandClamp.vertices');

% 第二个T模块
UpdateTM(hR(8:9),DrawCor,-JAs(3));

% 移动到第二个T模块的末端
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(3))*tl(0,0,93.4);
% 卡环
UpdateVs(hR(10),DrawCor,BandClamp.vertices');

%---- 5/6自由度不同的部分 ----%
if robot.DoFs == 6
    % 增加的I模块
    DrawCor = DrawCor*tl(0,0,163.4)*rz(JAs(4));
    UpdateVs(hR(11),DrawCor,IModule.vertices');  
    JANum = 5;
else
    % 加长套筒
    UpdateVs(hR(11),DrawCor,HollowPart.vertices');
    % 移动到套筒末端
    DrawCor = DrawCor*tl(0,0,53.2); 
    JANum = 4;
end

% 卡环
UpdateVs(hR(12),DrawCor,BandClamp.vertices');

% 第三个T模块
UpdateTM(hR(13:14),DrawCor,-JAs(JANum));

% 移动到第三个T模块末端
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(JANum))*tl(0,0,93.4);
% 卡环
UpdateVs(hR(15),DrawCor,BandClamp.vertices');

% 移动到第二个I模块末端
DrawCor = DrawCor*tl(0,0,163.4);
% I模块
UpdateVs(hR(16),DrawCor,IModule.vertices');
% 卡环
UpdateVs(hR(17),DrawCor,BandClamp.vertices');
% 第二个夹持器
UpdateVs(hR(18),DrawCor*rz(JAs(JANum+1))*ry(180),SModule.vertices');


end
% 刷新画面
drawnow;
end

%% 更新各零部件的顶点位置
% 更新T关节的顶点
function UpdateTM(hT,T,Angle)

global TModuleP1 TModuleP2;

% 第一部分
UpdateVs(hT(1),T,TModuleP1.vertices');
% 第二部分
T = T*tl(0,0,146.6)*ry(Angle);
UpdateVs(hT(2),T,TModuleP2.vertices');

end

function UpdateVs(hPart,T,V)

V = [V(1,:); V(2,:); V(3,:); ones(1,length(V))];
V = T*V;
set(hPart,'Vertices',V(1:3,:)');

end

% Homogeneous manipulation functions follow:
function Ry = ry(THETA)
% ROTATION ABOUT THE Y-AXIS
% NOTE: The angel THETA must be in DEGREES.
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Ry = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
%
function Rz = rz(THETA)
% ROTATION ABOUT THE Z-AXIS
% NOTE:  The angle THETA must be in DEGREES.
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Rz = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end

function T = tl(x,y,z)
% TRANSLATION ALONG THE X, Y, AND Z AXES
T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
end