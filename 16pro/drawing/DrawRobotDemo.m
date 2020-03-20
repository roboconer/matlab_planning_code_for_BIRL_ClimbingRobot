global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;

[BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();

% InitOri = [0,-90,-90];    %��λdeg
% InitPos = [0,0,0];
%CurHMatrix  = CalTMatrix(InitOri,InitPos);
[tr,an]= robot_configuration(x);
num_robot = size(an,1);
 CurHMatrix  = eye(4);
% CurHMatrix(1:3,4) = [7815,3585,145.4]';

% ע�⣡����Ϊ�������˵���λ����ɹ��ͣ���ͼʱĬ��robot.TarJAs - [0,90,0,90,0]
% 'TarJAs',[0,0,0,0,0]��G1���£�G2����ʱ�ĹؽڽǶ�ֵ��
% 'TarJAs',[0,180,0,180,0] G1���ϣ�G2����ʱ�ĹؽڽǶ�ֵ
Climbot5D = struct('DoFs',5,'LinksLen',[340.7,293.2,293.2,340.7],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
    'TarJAs',[0,90,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);

for i=1:num_robot
    %Climbot5D.LinksLen = x(2*i-1,:)*1000;
    %Climbot5D.TarJAs = an(i,:);
    Climbot5D.CurGraspMtx = tr(:,:,i);
    Climbot5D.CurGraspMtx(1:3,4) = 1000*x(2*i-1,:)';
    Climbot5D.TarJAs(2:4) = an(i,2:4);
     DrawRobot(Climbot5D,1);
end
% ע�⣡����Ϊ�������˵���λ����ɹ��ͣ���ͼʱĬ��robot.TarJAs - [0,90,90,0,0,0]
% 'TarJAs',[0,0,90,0,-90,0] G1���£�G2����ʱ�ĹؽڽǶ�ֵ
% 'TarJAs',[0,180,90,0,90,0] G1���ϣ�G2����ʱ�ĹؽڽǶ�ֵ
% Climbot6D = struct('DoFs',6,'LinksLen',[436.3,293.2,213.1,190.3,436.3],'ModuleRadii',50,...
%     'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,6),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%     'TarJAs',[0,0,90,0,-90,0],'hIKine',@IKine6D,'hLink',@Linkage6D); 

% Climbot5D = struct('DoFs',5,'LinksLen',[436.3,293.2,293.2,436.3],'ModuleRadii',50,...
%     'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%     'TarJAs',zeros(1,5),'hIKine',@IKine5D,'hLink',@Linkage5D);
% Climbot6D = struct('DoFs',6,'LinksLen',[436.3,293.2,213.1,190.3,436.3],'ModuleRadii',50,...
%     'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,6),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%     'TarJAs',zeros(1,6),'hIKine',@IKine6D,'hLink',@Linkage6D);  % 'TarJAs',[0,135,90,0,135,0]
% 
% % ���
% Truss = [1000,0,0, 1000,0,1700, 0, 0;              % ��ǰ�гָ�
%     1800,-300,1400, 700,1400,1900, 0, 0;];    % Ŀ��гָ�
% 
% % ָ����ǰ�гָ˺�Ŀ��ˣ���������ϰ��ˣ��ɼӵ�Truss���м�
% cpi = 1;
% gpi = size(Truss,1);
% 
% % ָ����ǰ�г�λ��   ������ͨ����ʽ��60�ı�����˵ļг�ƽ��
% Climbot6D.CurGraspMtx = CalGraspMtx(Truss(cpi,:),Truss(cpi,8),0.6,1,[]);
% Climbot6D.TarGraspMtx = CalGraspMtx(Truss(gpi,:),Truss(gpi,8),0.5,2,Climbot6D.CurGraspMtx);

%Climbot5D.TarGraspMtx(1:3,1:3) = [1 0 0; 0 -1 0; 0 0 -1];
%Climbot5D.TarGraspMtx(1,4)=586;

% Climbot6D.TarGraspMtx(1:3,1:3) = [1 0 0; 0 -1 0; 0 0 -1];
% Climbot6D.TarGraspMtx(1,4)=696.5;


%hRobot = DrawRobot(Climbot5D,1);
% hold on;
% hRobot2 = DrawRobot(Climbot6D,1,1);

%light                               % add a default light
%lightangle(-45,30)
material metal
daspect([1 1 1])                    % Setting the aspect ratio
view(3)                             % Isometric view
xlabel('X'),ylabel('Y'),zlabel('Z');
%axis([-500,500,-500,500,-500,1200]);
%axis off;

% pause(0.5)
% Climbot6D.TarJAs = [0,45,0,45,90,0];
% MoveRobot(hRobot,Climbot6D,1);



