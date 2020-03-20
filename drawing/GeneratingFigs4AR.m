%% Reachable grasping regions 2 Advanced Robotics

close all;

format long g

% 以下两行代码只需要运行一次
global BandClamp Gripper HollowPart IModuleP1 IModuleP2 TModuleP1 TModuleP2 TModuleP3;
[BandClamp,Gripper,HollowPart,IModuleP1,IModuleP2,TModuleP1,TModuleP2,TModuleP3] = readAllParts();

% 修改FigNum来选择需要产生的图
% ！！！每次只能产生一个，文件默认保存为.png格式，位于'E:\4P1M\Papers\Climbot\RGRs2AR\....png’下
% ！！！有些图超出范围，需要稍微调整一下坐标标注
% ！！！有些图比较暗，可以适当改变光源角度

FigNum = 5;
% 1 == 相交圆杆，5DoFs，机器人平面与杆件平面重合
% 2 == 相交圆杆，5DoFs，机器人平面与杆件平面不重合
% 3 == 平行圆杆，5DoFs
% 4 == 平行方杆，5DoFs，唯一解
% 5 == 相错圆杆，5DoFs
% 6 == 相错方杆，5DoFs，唯一解
% 7 == 相交方杆（绕自身中心轴转动），6DoFs
% 8 == 平行方杆，6DoFs
% 9 == 相错方杆，6DoFs
% 10 == 相同杆，6DoFs

GenerateFile = 0;
% 有箭头坐标表示夹持姿态的都不建议直接生成，适当调整标注后再从command window输入export_fig ...

 %% 机器人初始状态
 % 在全局中的位姿
% InitOri = [0,-90,-90];    %单位deg
% InitPos = [1000,0,1400];
InitOri = [0,-90,-90];    %单位deg
InitPos = [0,0,0];
CurHMatrix  = CalTMatrix(InitOri,InitPos);

Climbot5D = struct('DoFs',5,'LinksLen',[389.12,375,375,389.12],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
    'TarJAs',zeros(1,5),'hIKine',@IKine5D,'hLink',@Linkage5D);
Climbot6D = struct('DoFs',6,'LinksLen',[389.12,375,56.5,318.5,389.12],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',[0,180,90,0,90,0],'FixedGripper',2,'TarGraspMtx',eye(4,4),...
    'TarJAs',[0,180,90,0,90,0],'hIKine',@IKine6D,'hLink',@Linkage6D);

% 自由度设置
if FigNum < 7 % 5DoFs
    robot = Climbot5D;
else % 6DoFs
    robot = Climbot6D;
end

% 杆件桁架
Truss = [
    %图1#
%     1000,0,0, 1000,0,2300, 0, 0;              % 当前夹持杆
%     1000,-300,1900, 1000,1700,1500, 0, 0;     % 锐角圆杆、夹持点不同
    0,0,0, 0,0,2300, 0, 0;              % 当前夹持杆
    0,-300,500, 0,1700,750, 0, 0;     % 锐角圆杆、夹持点不同
    %图2#
    1000,0,0, 1000,0,1500, 0, 0;              % 当前夹持杆
    1000,100,1700, 1000,1700,2000, 0, 0;     % 锐角圆杆、夹持点不同
    %图3#
    1000,500,0, 1000,500,2000, 0, 0;              % 当前夹持杆
    1200,1150,0, 1200,1150,2300, 0, 0;              % 与第一根当前夹持杆平行，5DoFs,圆,姿态绕目标杆无转动,正常 - 竖直  
    %图4#
    1000,300,0, 1000,300,2000, 0, 0;              % 当前夹持杆
    1200,1200,0, 1200,1200,2000, 1, -30;              % 与第一根当前夹持杆平行，5DoFs,圆,姿态绕目标杆无转动,正常 - 竖直 
    %图5#
    1000,0,0, 1000,0,1700, 0, 0;              % 当前夹持杆
    1800,-300,1400, 700,1400,1900, 0, 0;        % 圆杆,5DoFs,无数解
    %图6#
    1000,0,0, 1000,0,1700, 0, 0;              % 当前夹持杆
    1800,-300,1400, 700,1400,1900, 1, 0;        % 方杆,5DoFs,唯一解
    %图7#
    1000,0,0, 1000,0,1700, 0, 0;              % 当前夹持杆
    1000,-300,1500, 1000,1300,1850, 1, 20;           % 方杆
    %图8#
    800,-300,1500, 800,1700,1000, 1, 0;          % 当前夹持杆，倾斜
    1400,-300,1500, 1400,1700,1000, 1, 30;       % 与当前夹持杆平行，6DoFs,方,定,正常
    %图9#
    1000,0,0, 1000,0,1700, 0, 0;              % 当前夹持杆
    1800,-300,1400, 700,1400,1900, 1, 0;
    %1400,-300,1400, 1600,1300,1900, 1, 7;        % 方杆,6DoFs,定姿态 - 夹持空间
    
    1000,0,0, 1000,0,2000, 0, 0;              % 当前夹持杆
    1000,0,1000, 1000,0,2000, 1, 45;     % 锐角圆杆、夹持点不同
    ];

Pole_radii = 30;

switch FigNum
    
    %% 子图1======== 相交杆---圆杆  
    case 1
        
        figure;
        
        CurHMtx = [6.12323399573677e-17,1,6.12323399573677e-17,0;0,-6.12323399573677e-17,1,0;1,-6.12323399573677e-17,-3.74939945665464e-33,1000;0,0,0,1;];
%         CurHMtx(3,4) = CurHMtx(3,4) - 460;%460
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [0,0,0, 0,0,2000, 0, 0;        % 当前夹持杆
            0,-300,1600, 0,1500,1950, 0, 0];     % 目标夹持杆
%         truss = [Truss(1,:);Truss(2,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
       
    %% 子图2======== 相交杆---圆杆
    case 2
        
        figure;
        
        InitOri = [0,-90,-60];    %单位deg  % 不在相交面上
        InitPos = [1000,0,800];
        CurHMtx = CalTMatrix(InitOri,InitPos);
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [Truss(3,:);Truss(4,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
        
    %% 子图3======= 平行杆---圆杆
    case 3
        
        figure;
        
        truss = [Truss(5,:);Truss(6,:)];
        InitOri = [180,-90,0];    %单位deg
        InitPos = [1000,500,990];
        CurHMtx = CalTMatrix(InitOri,InitPos);
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
 
        
   %% 子图4======= 平行杆 圆杆to圆杆     
    case 4

        figure;
        
        truss = [Truss(7,:);Truss(8,:)];
        InitOri = [180,-90,0];    %单位deg
        InitPos = [1000,300,990];
        CurHMtx = CalTMatrix(InitOri,InitPos);
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
        
   %% 子图5======== 相错杆--to圆杆
    case 5
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 300;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [Truss(9,:);Truss(10,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
       
        
   %% 子图6======== 相错杆--to方杆
    case 6
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 300;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [Truss(11,:);Truss(12,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
        
    %% 子图7 ======== 相交杆
    case 7
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 500;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [Truss(13,:);Truss(14,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
       
   %% 子图8======= 平行杆   
    case 8
        
        figure;
        
        truss = [Truss(15,:);Truss(16,:)];
        % 注意，以下只是暂缓之策，CurMtx是G2的夹持姿态，要变换成G1的
        CurRMtx = graspPose(truss(1,:),1);
        Pos = truss(1,1:3) + (truss(1,4:6)-truss(1,1:3))*0.5;  % 当前夹持点
        CurHMtx  = [CurRMtx,transpose(Pos);0 0 0 1];
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
        
   %% 子图9======== 相错杆        
    case 9
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 200;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [Truss(17,:);Truss(18,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
        
        %% 子图10======== 相错杆 
    case 10
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(3,4) = CurHMtx(3,4) - 600;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [Truss(19,:);Truss(20,:)];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        truss(2,7)=0;
        
        % 先把场景和机器人初始状态显示出来
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
        [searchFlag,GraspBeg,GraspEnd] = searchGraspSapce(robot,truss,2,false,1,1); % 注意相同杆不能做碰撞检测！！！
        
        CurHMtx = CurHMatrix;
        CurHMtx(2,4)=1500;
        CurHMtx(3,4) = CurHMtx(3,4) - 600;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [1000,1500,0, 1000,1500,2000, 1, 0;              % 当前夹持杆
            1000,1500,1000, 1000,1500,2000, 1, 0;];
        if truss(2,7)
            TarOriMtx = graspPose(truss(2,:),2);
            % 显示
            drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
            % 转换到机器人基座标系下
            TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
        else
            TarOriMtx = [];
        end
        
        % 先把场景和机器人初始状态显示出来
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
        
        [searchFlag,GraspBeg,GraspEnd] = searchGraspSapce(robot,truss,2,false,1,1);% 注意相同杆不能做碰撞检测！！！
        
        
    case 11
        
        figure;
        
        CurHMtx = CurHMatrix;
        CurHMtx(1,4)=0;
        CurHMtx(3,4) = CurHMtx(3,4) - 1400;
        
        zmax = 1000;
        zdelta = 200;
        
        % 杆件环境
        % 方杆的话，要指定夹持姿态
        truss = [0,0,-1000, 0,0,1200, 1, 0;              % 当前夹持杆
            600,-500,zmax, 600,1200,zmax, 1, 0;];
        
        drawcube(truss(1,1:3),truss(1,4:6),Pole_radii+10,truss(1,8),'y');
        
        while zmax > -200
            
            if truss(2,7)
                TarOriMtx = graspPose(truss(2,:),2);
                % 显示
                %drawcoordinate([TarOriMtx,transpose(truss(2,4:6));0 0 0 1],[],[0 0 0],'noa');
                % 转换到机器人基座标系下
                TarOriMtx = CurHMtx(1:3,1:3) \ TarOriMtx;
            else
                TarOriMtx = [];
            end
            
            % 先把场景和机器人初始状态显示出来
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


% 图公共属性设置
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