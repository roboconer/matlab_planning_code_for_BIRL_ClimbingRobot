%% Reachable grasping regions 2 Advanced Robotics

close all;

format long g

% �������д���ֻ��Ҫ����һ��
global BandClamp Gripper HollowPart IModuleP1 IModuleP2 TModuleP1 TModuleP2 TModuleP3;
[BandClamp,Gripper,HollowPart,IModuleP1,IModuleP2,TModuleP1,TModuleP2,TModuleP3] = readAllParts();

% �޸�FigNum��ѡ����Ҫ������ͼ
% ������ÿ��ֻ�ܲ���һ�����ļ�Ĭ�ϱ���Ϊ.png��ʽ��λ��'E:\4P1M\Papers\Climbot\RGRs2AR\....png����
% ��������Щͼ������Χ����Ҫ��΢����һ�������ע
% ��������Щͼ�Ƚϰ��������ʵ��ı��Դ�Ƕ�

FigNum = 5;
% 1 == �ཻԲ�ˣ�5DoFs��������ƽ����˼�ƽ���غ�
% 2 == �ཻԲ�ˣ�5DoFs��������ƽ����˼�ƽ�治�غ�
% 3 == ƽ��Բ�ˣ�5DoFs
% 4 == ƽ�з��ˣ�5DoFs��Ψһ��
% 5 == ���Բ�ˣ�5DoFs
% 6 == ����ˣ�5DoFs��Ψһ��
% 7 == �ཻ���ˣ�������������ת������6DoFs
% 8 == ƽ�з��ˣ�6DoFs
% 9 == ����ˣ�6DoFs
% 10 == ��ͬ�ˣ�6DoFs

GenerateFile = 0;
% �м�ͷ�����ʾ�г���̬�Ķ�������ֱ�����ɣ��ʵ�������ע���ٴ�command window����export_fig ...

 %% �����˳�ʼ״̬
 % ��ȫ���е�λ��
% InitOri = [0,-90,-90];    %��λdeg
% InitPos = [1000,0,1400];
InitOri = [0,-90,-90];    %��λdeg
InitPos = [0,0,0];
CurHMatrix  = CalTMatrix(InitOri,InitPos);

Climbot5D = struct('DoFs',5,'LinksLen',[389.12,375,375,389.12],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
    'TarJAs',zeros(1,5),'hIKine',@IKine5D,'hLink',@Linkage5D);
Climbot6D = struct('DoFs',6,'LinksLen',[389.12,375,56.5,318.5,389.12],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',[0,180,90,0,90,0],'FixedGripper',2,'TarGraspMtx',eye(4,4),...
    'TarJAs',[0,180,90,0,90,0],'hIKine',@IKine6D,'hLink',@Linkage6D);

% ���ɶ�����
if FigNum < 7 % 5DoFs
    robot = Climbot5D;
else % 6DoFs
    robot = Climbot6D;
end

% �˼����
Truss = [
    %ͼ1#
%     1000,0,0, 1000,0,2300, 0, 0;              % ��ǰ�гָ�
%     1000,-300,1900, 1000,1700,1500, 0, 0;     % ���Բ�ˡ��гֵ㲻ͬ
    0,0,0, 0,0,2300, 0, 0;              % ��ǰ�гָ�
    0,-300,500, 0,1700,750, 0, 0;     % ���Բ�ˡ��гֵ㲻ͬ
    %ͼ2#
    1000,0,0, 1000,0,1500, 0, 0;              % ��ǰ�гָ�
    1000,100,1700, 1000,1700,2000, 0, 0;     % ���Բ�ˡ��гֵ㲻ͬ
    %ͼ3#
    1000,500,0, 1000,500,2000, 0, 0;              % ��ǰ�гָ�
    1200,1150,0, 1200,1150,2300, 0, 0;              % ���һ����ǰ�гָ�ƽ�У�5DoFs,Բ,��̬��Ŀ�����ת��,���� - ��ֱ  
    %ͼ4#
    1000,300,0, 1000,300,2000, 0, 0;              % ��ǰ�гָ�
    1200,1200,0, 1200,1200,2000, 1, -30;              % ���һ����ǰ�гָ�ƽ�У�5DoFs,Բ,��̬��Ŀ�����ת��,���� - ��ֱ 
    %ͼ5#
    1000,0,0, 1000,0,1700, 0, 0;              % ��ǰ�гָ�
    1800,-300,1400, 700,1400,1900, 0, 0;        % Բ��,5DoFs,������
    %ͼ6#
    1000,0,0, 1000,0,1700, 0, 0;              % ��ǰ�гָ�
    1800,-300,1400, 700,1400,1900, 1, 0;        % ����,5DoFs,Ψһ��
    %ͼ7#
    1000,0,0, 1000,0,1700, 0, 0;              % ��ǰ�гָ�
    1000,-300,1500, 1000,1300,1850, 1, 20;           % ����
    %ͼ8#
    800,-300,1500, 800,1700,1000, 1, 0;          % ��ǰ�гָˣ���б
    1400,-300,1500, 1400,1700,1000, 1, 30;       % �뵱ǰ�гָ�ƽ�У�6DoFs,��,��,����
    %ͼ9#
    1000,0,0, 1000,0,1700, 0, 0;              % ��ǰ�гָ�
    1800,-300,1400, 700,1400,1900, 1, 0;
    %1400,-300,1400, 1600,1300,1900, 1, 7;        % ����,6DoFs,����̬ - �гֿռ�
    
    1000,0,0, 1000,0,2000, 0, 0;              % ��ǰ�гָ�
    1000,0,1000, 1000,0,2000, 1, 45;     % ���Բ�ˡ��гֵ㲻ͬ
    ];

Pole_radii = 30;

switch FigNum
    
    %% ��ͼ1======== �ཻ��---Բ��  
    case 1
        
        figure;
        
        CurHMtx = [6.12323399573677e-17,1,6.12323399573677e-17,0;0,-6.12323399573677e-17,1,0;1,-6.12323399573677e-17,-3.74939945665464e-33,1000;0,0,0,1;];
%         CurHMtx(3,4) = CurHMtx(3,4) - 460;%460
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [0,0,0, 0,0,2000, 0, 0;        % ��ǰ�гָ�
            0,-300,1600, 0,1500,1950, 0, 0];     % Ŀ��гָ�
%         truss = [Truss(1,:);Truss(2,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,true);
       
    %% ��ͼ2======== �ཻ��---Բ��
    case 2
        
        figure;
        
        InitOri = [0,-90,-60];    %��λdeg  % �����ཻ����
        InitPos = [1000,0,800];
        CurHMtx = CalTMatrix(InitOri,InitPos);
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [Truss(3,:);Truss(4,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),2*Pole_radii,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
        
        
    %% ��ͼ3======= ƽ�и�---Բ��
    case 3
        
        figure;
        
        truss = [Truss(5,:);Truss(6,:)];
        InitOri = [180,-90,0];    %��λdeg
        InitPos = [1000,500,990];
        CurHMtx = CalTMatrix(InitOri,InitPos);
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
 
        
   %% ��ͼ4======= ƽ�и� Բ��toԲ��     
    case 4

        figure;
        
        truss = [Truss(7,:);Truss(8,:)];
        InitOri = [180,-90,0];    %��λdeg
        InitPos = [1000,300,990];
        CurHMtx = CalTMatrix(InitOri,InitPos);
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
        
        
   %% ��ͼ5======== ����--toԲ��
    case 5
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 300;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [Truss(9,:);Truss(10,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
       
        
   %% ��ͼ6======== ����--to����
    case 6
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 300;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [Truss(11,:);Truss(12,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
        
        
    %% ��ͼ7 ======== �ཻ��
    case 7
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 500;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [Truss(13,:);Truss(14,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
        
       
   %% ��ͼ8======= ƽ�и�   
    case 8
        
        figure;
        
        truss = [Truss(15,:);Truss(16,:)];
        % ע�⣬����ֻ���ݻ�֮�ߣ�CurMtx��G2�ļг���̬��Ҫ�任��G1��
        CurRMtx = graspPose(truss(1,:),1);
        Pos = truss(1,1:3) + (truss(1,4:6)-truss(1,1:3))*0.5;  % ��ǰ�гֵ�
        CurHMtx  = [CurRMtx,transpose(Pos);0 0 0 1];
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
        
        
   %% ��ͼ9======== ����        
    case 9
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 200;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [Truss(17,:);Truss(18,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd,tt] = searchGraspSapce(robot,truss,2,true,1,1);
        
        
        %% ��ͼ10======== ���� 
    case 10
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 600;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [Truss(19,:);Truss(20,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        truss(2,7)=0;
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:1%TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        [searchFlag,GraspBeg,GraspEnd] = searchGraspSapce(robot,truss,2,false,1,1); % ע����ͬ�˲�������ײ��⣡����
        
        CurHMtx = CurHMatrix;
        CurHMtx(2,4)=1500;
        CurHMtx(3,4) = CurHMtx(3,4) - 600;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [1000,1500,0, 1000,1500,2000, 1, 0;              % ��ǰ�гָ�
            1000,1500,1000, 1000,1500,2000, 1, 0;];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % ��ʾ
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % ת���������˻�����ϵ��
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
        TotalPoleNum = size(truss,1);
        for j =1:1%TotalPoleNum
            if truss(j,7)
                drawcube(truss(j,1:3),truss(j,4:6),Pole_radii+10,truss(j,8),'y');
            else
                drawcylinder(truss(j,1:3),truss(j,4:6),Pole_radii,'b',1,1);
            end
        end
        
        robot.CurGraspMtx = CurHMtx;
        if isempty(TarOriMtx)
            robot.TarGraspMtx = [];
        else
            robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
        end
        
        [searchFlag,GraspBeg,GraspEnd] = searchGraspSapce(robot,truss,2,false,1,1);% ע����ͬ�˲�������ײ��⣡����
        
        
    case 11
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(1,4)=0;
        CurHMtx(3,4) = CurHMtx(3,4) - 1400;
        
        zmax = 1000;
        zdelta = 200;
        
        % �˼�����
        % ���˵Ļ���Ҫָ���г���̬
        truss = [0,0,-1000, 0,0,1200, 1, 0;              % ��ǰ�гָ�
            600,-500,zmax, 600,1200,zmax, 1, 0;];
        
        drawcube(truss(1,1:3),truss(1,4:6),Pole_radii+10,truss(1,8),'y');
        
        while zmax > -200
            
            if truss(2,7)
                TarOriMtx = graspPose(truss(2,:),2);
                % ��ʾ
                %drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
                % ת���������˻�����ϵ��
                TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
            else
                TarOriMtx = [];
            end
            
            % �Ȱѳ����ͻ����˳�ʼ״̬��ʾ����
            drawcube(truss(2,1:3),truss(2,4:6),Pole_radii+10,truss(2,8),'y');

            robot.CurGraspMtx = CurHMtx;
            if isempty(TarOriMtx)
                robot.TarGraspMtx = [];
            else
                robot.TarGraspMtx(1:3,1:3) = TarOriMtx;
            end
            
            [searchFlag,GraspBeg,GraspEnd] = searchGraspSapce(robot,truss,2,true,1,1);
            
            zmax = zmax - zdelta;
            truss(2,:) = [600,-500,zmax, 600,1200,zmax, 1, 0];
            
        end
        
        xmax = 1000;
        xdelta = 200;
        
        daspect([1 1 1])
        xlabel('X'),ylabel('Y'),zlabel('Z');
        
end          


% ͼ������������
grid on;
axis equal;
% axis([500,1500,-500,2000,0,2500]);
axis([500,1500,-500,2000,-500,2500]);
xlabel('\it{x}\rm(\it{mm}\rm)','FontSize',24,'FontName','Times New Roman');
ylabel('\it{y}\rm(\it{mm}\rm)','Fontsize',24,'FontName','Times New Roman');
zlabel('\it{z}\rm(\it{mm}\rm)','Fontsize',24,'FontName','Times New Roman');
set(gca,'linewidth',1.5);
set(gca,'fontsize',22);
set(gca,'Fontname','Times New Roman');
set(gcf,'Units','inches','Position',[4 0.8 8 8*88/98]);


switch FigNum
    case 1
        view(75,20);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Crossover5Da.png' -opengl -transparent -r200;
        end
    case 2
        view(75,20);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Crossover5Db.png' -opengl -transparent -r200;
        end
    case 3
        view(75,20);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Parallel5Da.png' -opengl -transparent -r200;
        end
    case 4
        view(75,20);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Parallel5Db.png' -opengl -transparent -r200;
        end
    case 5
        view(-75,30);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Staggered5Da.png' -opengl -transparent -r200;
        end
    case 6
        view(-75,30);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Staggered5Db.png' -opengl -transparent -r200;
        end
    case 7
        view(70,20);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Crossover6D.png' -opengl -transparent -r200;
        end
    case 8
        view(-105,25);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Parallel6D.png' -opengl -transparent -r200;
        end
    case 9
        view(-75,30);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Staggered6D.png' -opengl -transparent -r200;
        end
    case 10
        view(-110,20);
        if GenerateFile
            export_fig 'E:\4P1M\Papers\Climbot\RGRs2AR\Same6D.png' -opengl -transparent -r200;
        end
    case 11
         axis([-1500,500,-800,1500,-500,1500]);
        view(-75,30);
end