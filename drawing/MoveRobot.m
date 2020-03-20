%% ==============================================================
% ʹ������һ�˼г���һ���˶��ĺ���
% ���룺hR - �����˸����˾����Ϊ1*24����DrawRobot�л��
%       robot - �����˽ṹ��
%       mode - ָʾ�Ƿ���Ҫ�Ƚ���������ı�־λ��ʡ�Ի���Ϊ�ն����������⣩
% �����
function MoveRobot(hR,robot,mode)

% ============= �����Ƿ�Ҫ����� ===============
% if (nargin < 3) || isempty(mode)
%     % �������ǻ����˵�ĩ��λ�ˣ����û����˵���Ϣ����������
%     [ikflag,robot.TarJAs] = robot.hIKine(robot);
%     if ~ikflag
%         disp('û����⣬��ʵ��û����...');
%         return;
%     end
% end

% ======== �����������λ��ؽ���λ��ͳһ������ ==========
if robot.DoFs == 5
    JAs = robot.TarJAs - [0,90,0,90,0];
elseif robot.DoFs == 6
    JAs = robot.TarJAs - [0,90,90,0,0,0];
else
    error('ֻ�ܻ�5���ɶȻ�6���ɶȵ�����������');
end

% ======== �������гֶ˵����� ==========
% if robot.FixedGripper == 2
%     % ԭ����Ŀ������ϵ�������ڵĻ�����ϵ����������ϵ��
%     robot.CurGraspMtx = robot.CurGraspMtx * robot.TarGraspMtx;
%     % �����гֶ�ʱ�����ݽ�����ϵ�Ĺ���y��z�ᷴת
%     robot.CurGraspMtx(1:3,2:3) = [-robot.CurGraspMtx(1:3,2),-robot.CurGraspMtx(1:3,3)];
% end

% ======== ���¿�ʼ�ƶ������˵ĸ����㲿�� ==========
global BandClamp HollowPart IModule SModule;

% �ƶ����г���ԭ��׼����ʼ��ͼ
if robot.FixedGripper == 2
    
    DrawCor = robot.CurGraspMtx*tl(0,0,85);
    % ��һ���г���
    UpdateVs(hR(1),DrawCor,SModule.vertices');
    % ����
    UpdateVs(hR(2),DrawCor,BandClamp.vertices');
     % �ƶ���Iģ���ĩ��
    DrawCor = DrawCor*tl(0,0,163.4);
    % ��һ��Iģ��
    UpdateVs(hR(3),DrawCor,IModule.vertices');
    
   
    % ����
    UpdateVs(hR(4),DrawCor,BandClamp.vertices');
    
    % �ƶ�����һ��Tģ��͵ڶ���Tģ��Ľ�ϴ�
    DrawCor = DrawCor*rz(JAs(1)+180)*tl(0,0,93.4)*ry(JAs(2))*tl(0,0,146.6);
    % ��һ��Tģ��
    UpdateTM(hR(5:6),DrawCor*ry(180),-JAs(2));
    
    % ����
    UpdateVs(hR(7),DrawCor,BandClamp.vertices');
    JANum = 4;
     % �ӳ���Ͳ
    UpdateVs(hR(11),DrawCor,HollowPart.vertices');
    % �ƶ�����Ͳĩ��
    DrawCor = DrawCor*tl(0,0,53.2); 
      %����
    UpdateVs(hR(10),DrawCor,BandClamp.vertices');
    % �ƶ����ڶ���Tģ���ĩ��
    DrawCor = DrawCor*tl(0,0,93.4)*ry(JAs(3))*tl(0,0,146.6);
    
    % �ڶ���Tģ��
    UpdateTM(hR(8:9),DrawCor*ry(180),-JAs(3));
    
    % ����
    UpdateVs(hR(12),DrawCor,BandClamp.vertices');

    % ������Tģ��
    UpdateTM(hR(13:14),DrawCor,JAs(JANum));

    % �ƶ���������Tģ��ĩ��
    DrawCor = DrawCor*tl(0,0,146.6)*ry(JAs(JANum))*tl(0,0,93.4);
    % ����
    UpdateVs(hR(15),DrawCor,BandClamp.vertices');

    % �ƶ����ڶ���Iģ��ĩ��
    DrawCor = DrawCor*tl(0,0,163.4);
    % Iģ��
    UpdateVs(hR(16),DrawCor,IModule.vertices');
    % ����
    UpdateVs(hR(17),DrawCor,BandClamp.vertices');
    % �ڶ����г���
    UpdateVs(hR(18),DrawCor*rz(JAs(JANum+1)+180)*ry(180),SModule.vertices');

    % ˢ�»���
  %  drawnow;
    
end
  
if robot.FixedGripper == 1
DrawCor = robot.CurGraspMtx*tl(0,0,85);   % ��ͼ����
% ��һ���г���
 UpdateVs(hR(1),DrawCor,SModule.vertices');
% ����
UpdateVs(hR(2),DrawCor,BandClamp.vertices');

% �ƶ���Iģ���ĩ��
DrawCor = DrawCor*tl(0,0,163.4);
% ��һ��Iģ��
 UpdateVs(hR(3),DrawCor,IModule.vertices');

% ����
UpdateVs(hR(4),DrawCor,BandClamp.vertices');

% �ƶ�����һ��Tģ��͵ڶ���Tģ��Ľ�ϴ�
DrawCor = DrawCor*rz(JAs(1))*tl(0,0,93.4)*ry(-JAs(2))*tl(0,0,146.6);

% ��һ��Tģ��
UpdateTM(hR(5:6),DrawCor*ry(180),JAs(2));

% ����
UpdateVs(hR(7),DrawCor,BandClamp.vertices');

% �ڶ���Tģ��
UpdateTM(hR(8:9),DrawCor,-JAs(3));

% �ƶ����ڶ���Tģ���ĩ��
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(3))*tl(0,0,93.4);
% ����
UpdateVs(hR(10),DrawCor,BandClamp.vertices');

%---- 5/6���ɶȲ�ͬ�Ĳ��� ----%
if robot.DoFs == 6
    % ���ӵ�Iģ��
    DrawCor = DrawCor*tl(0,0,163.4)*rz(JAs(4));
    UpdateVs(hR(11),DrawCor,IModule.vertices');  
    JANum = 5;
else
    % �ӳ���Ͳ
    UpdateVs(hR(11),DrawCor,HollowPart.vertices');
    % �ƶ�����Ͳĩ��
    DrawCor = DrawCor*tl(0,0,53.2); 
    JANum = 4;
end

% ����
UpdateVs(hR(12),DrawCor,BandClamp.vertices');

% ������Tģ��
UpdateTM(hR(13:14),DrawCor,-JAs(JANum));

% �ƶ���������Tģ��ĩ��
DrawCor = DrawCor*tl(0,0,146.6)*ry(-JAs(JANum))*tl(0,0,93.4);
% ����
UpdateVs(hR(15),DrawCor,BandClamp.vertices');

% �ƶ����ڶ���Iģ��ĩ��
DrawCor = DrawCor*tl(0,0,163.4);
% Iģ��
UpdateVs(hR(16),DrawCor,IModule.vertices');
% ����
UpdateVs(hR(17),DrawCor,BandClamp.vertices');
% �ڶ����г���
UpdateVs(hR(18),DrawCor*rz(JAs(JANum+1))*ry(180),SModule.vertices');


end
% ˢ�»���
drawnow;
end

%% ���¸��㲿���Ķ���λ��
% ����T�ؽڵĶ���
function UpdateTM(hT,T,Angle)

global TModuleP1 TModuleP2;

% ��һ����
UpdateVs(hT(1),T,TModuleP1.vertices');
% �ڶ�����
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