global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;

[BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();
 CurHMatrix  = eye(4);
figure(4);
Climbot5D = struct('DoFs',5,'LinksLen',[340.7,200,293.2,300],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
    'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
% Climbot5D = struct('DoFs',5,'LinksLen',[340.7,293.2,293.2,340.7],'ModuleRadii',50,...
%     'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
%     'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
%DrawRobotmo(Climbot5D,1);
Climbot5D.TarGraspMtx(1:3,2)=Climbot5D.TarGraspMtx(1:3,2)*-1;
Climbot5D.TarGraspMtx(1:3,3)=Climbot5D.TarGraspMtx(1:3,3)*-1;
 Climbot5D.TarGraspMtx(1:3,4) = [0.58 0 0]'*1000;
 circle( 0,0,580 );
 hold on;
circle( 0,0,400 );
  [Flag, Configs] = IKine5DNew(Climbot5D);
   
  Configs(5)=Configs(5)-90;
  Climbot5D.TarJAs =Configs;

  DrawRobotmo(Climbot5D,1);
%  Climbot5D.TarGraspMtx(1:3,4) = [-0.4 0.3 0]'*1000;
 % [Flag, Configs] = IKine5DNew(Climbot5D);
%     Configs(5) = Configs(5)+30;
%     Configs(1) = Configs(1)+30;
%   Climbot5D.TarJAs =Configs;

  
%   DrawRobotmo(Climbot5D,1);
   [ T,joi_p ] =  Kine5D( Configs ,4);
   T(1:3,2) = T(1:3,2)*-1;
   T(1:3,3) = T(1:3,3)*-1;
   
   Climbot5D.CurGraspMtx = T;
 %  [Flag, Configs] = IKine5DNew(Climbot5D);
 Configs = [Configs(5) Configs(4) Configs(3) Configs(2) Configs(1)];
  Climbot5D.TarJAs =Configs;
% Climbot5D.TarJAs = [180 10 -40 -10 0];
%   DrawRobotmo(Climbot5D,1);
% Climbot5D.TarJAs = [0 170 40 190 0];
 Climbot5D.FixedGripper=2;
  DrawRobotmo(Climbot5D,1);
   axis equal;
   axis([-3000,3000,-3000,3000,0, 3000]);