 function [  ] = joint_angle(joint_ang)
%UNTITLED 此处显示有关此函数的摘要
%   根据单步运动的关节角画机器人
%  Vnm=cross(M_sur(2,:)-M_sur(1,:),M_sur(3,:)-M_sur(2,:));   %计算多边形法向量
%     Vn_norm=norm(Vnm);
%     Vnm=Vnm/Vn_norm;
% 
%     bas_x = (M_sur(2,:)-M_sur(1,:))/norm(M_sur(2,:)-M_sur(1,:));   %求机器人运动的基坐标系
%     bas_z = Vnm(1,:);
%     bas_y = cross(bas_z,bas_x); 
%     bas_R = [bas_x',bas_y',bas_z'];
    
global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;
[BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();
 CurHMatrix  = eye(4);
 wrobot = struct('DoFs',5,'LinksLen',[340.7,293.2,293.2,340.7],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',[0 8.4728 -16.9456 8.4728 0],'FixedGripper',1,'TarGraspMtx',eye(4,4),...
    'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
%       wrobot.CurGraspMtx(1:3,1:3) = bas_R;
%        wrobot.CurGraspMtx(1:3,4)   = bap*1000;
% [p_s,con_v] = basemotion( stp,enp,bap );
% num_p = size(p_s,1);
% joint_ang = zeros(num_p,5);
% for i=1:num_p
%      wrobot.TarGraspMtx(1:3,4) = p_s(i,:)';
%      if i>1
%        wrobot.CurJAs=joint_ang(i-1,:);
%      end
%      [Flag, joint_ang(i,:)] = IKine5DNew(wrobot);
%   
%      
% end

  num_joi = size(joint_ang,1);
%   for i = 1: num_joi
%       [ T,joi_p(i,:) ] =  Kine5D( joi_ang(i,:) ,4)
%   end
%       plot3(joi_p(:,1), joi_p(:,2),joi_p(:,3),'-r','LineWidth',2);


num_r = floor(num_joi);
for i=1:num_r
%     wrobot.TarJAs = joint_ang(15*i,:);
%      DrawRobotmo(wrobot,1);
if joint_ang(i,1)<-188
    wrobot.TarJAs = joint_ang(i,:);
      DrawRobotmo(wrobot,0.3);
      i
      break;
end
end
for i=1:num_r
%     wrobot.TarJAs = joint_ang(15*i,:);
%      DrawRobotmo(wrobot,1);
if joint_ang(i,1)<-231
    wrobot.TarJAs = joint_ang(i,:);
      DrawRobotmo(wrobot,0.3);
      i
      break;
end
end
for i=1:num_r
%     wrobot.TarJAs = joint_ang(15*i,:);
%      DrawRobotmo(wrobot,1);
if joint_ang(i,1)<-274
    wrobot.TarJAs = joint_ang(i,:);
      DrawRobotmo(wrobot,0.3);
      i
      break;
end
end
 wrobot.TarJAs = joint_ang(6,:);
 DrawRobotmo(wrobot,0.3);
  wrobot.TarJAs = joint_ang(num_joi-6,:);
  DrawRobotmo(wrobot,1);
end
