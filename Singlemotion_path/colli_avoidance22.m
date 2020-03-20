function [ joint_ang] = colli_avoidance22( M_sur,stp,enp,bap,obs_p,so_option)
%UNTITLED2 此处显示有关此函数的摘要
%  生成单步无碰运动，针对机器人在不同壁面间过渡的情况,需要输入基座所在壁面，目标落足点所在壁面，起始落足点所在壁面
    %[ joint_ang,p_s ] = joint_angle(stp,enp,bap);
    global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;
    [BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();
 CurHMatrix  = eye(4);
  dis_warn = 0.05;  %注意距离为5cm
  dis_safe = 0.02;  %最小安全距离为2cm
  po1 = zeros(1,3);
  po2 = zeros(1,3);
  vec_lin = zeros(1,3);
  vec_eff = zeros(1,3);
  num_up = 0;
  enp_o = enp;
  stp_o = stp;
  bap_o = bap;
 wrobot = struct('DoFs',5,'LinksLen',[340.7,293.2,293.2,340.7],'ModuleRadii',50,...
    'CurGraspMtx',CurHMatrix,'CurJAs',[0   78.0230   10.1199  205.9813 0],'FixedGripper',1,'TarGraspMtx',eye(4,4),...
    'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);

 num_msur=size(M_sur,2);   %求机器人运动过程中需要经过的壁面及基座标系，并对环境基于基座标系作转化，设第一个壁面为基座标所在壁面，第二个壁面为目标落足点所在壁面，第三个壁面为起始落足点所在壁面
 m_sur = cell(num_msur,1);        %多边形点进行坐标系转换后的储存
 num_ob = floor(size(obs_p,1)/8);
obs_p2 = obs_p;

 for i=1:num_msur
    num_mp(i)=size(M_sur{i},1);
 end
 Vnm = zeros(num_msur,3);
for i=1:num_msur
    Vnm(i,:)=cross(M_sur{i}(2,:)-M_sur{i}(1,:),M_sur{i}(3,:)-M_sur{i}(2,:));   %计算多边形法向量
    Vn_norm=norm(Vnm(i,:));
    Vnm(i,:)=Vnm(i,:)/Vn_norm;
end
    bas_x = (M_sur{1}(2,:)-M_sur{1}(1,:))/norm(M_sur{1}(2,:)-M_sur{1}(1,:));   %求机器人运动的基坐标系
    bas_z = Vnm(1,:);
    bas_y = cross(bas_z,bas_x); 
    bas_R = [bas_x',bas_y',bas_z'];
    tar_x = (M_sur{2}(2,:)-M_sur{2}(1,:))/norm(M_sur{2}(2,:)-M_sur{2}(1,:));   %求机器人运动的目标坐标系
    tar_z = Vnm(2,:);
    tar_y = cross(tar_z,tar_x);
    tar_R = [tar_x',tar_y',tar_z'];
    tar_to_bas_R = bas_R'*tar_R;    %目标坐标系在基坐标系下的表示
     wor_to_bas_R = bas_R';   %世界坐标系在基座标系的表示
     wtoba_p = (-wor_to_bas_R*bap')';

    for i = 1:num_msur                  %求出机器人运动过程附着的壁面
        for j = 1:num_mp(i)
            m_sur{i}(j,:) = (wor_to_bas_R*M_sur{i}(j,:)')' + wtoba_p; 
        end
    end
 if num_ob>=1            %求基座标系下的障碍物
     for i = 1:size(obs_p,1)
         obs_p(i,:) = (wor_to_bas_R*obs_p(i,:)')'+ wtoba_p;
     end
         
 end
     enp = (wor_to_bas_R*enp')'+ wtoba_p;   %目标点在基座标系下的表示
     stp = (wor_to_bas_R*stp')'+ wtoba_p;   %起始落足点在基座标下的表示
     enp = enp + (wor_to_bas_R*Vnm(2,:)')'*0.05;       %目标吸附点的辅助路径径点

    sta_x = (M_sur{3}(2,:)-M_sur{3}(1,:))/norm(M_sur{3}(2,:)-M_sur{3}(1,:));   %求机器人运动的起点坐标系
    sta_z = Vnm(3,:);
    sta_y = cross(sta_z,sta_x);
    sta_R = [sta_x',sta_y',sta_z'];
    sta_to_bas_R = bas_R'*sta_R;    %起点坐标系在基坐标系下的表示
    sta_to_bas_R
     wrobot.TarGraspMtx(1:3,1:3) = sta_to_bas_R;
     Vn = (wor_to_bas_R*Vnm(3,:)')';
     wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
     wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向  
     wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
     [ Flag,sta_ang] = IKine5DNew(wrobot);                %求机器人起始构型的关节角
    %end
     
    wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人目标构型的关节角
    wrobot.CurJAs = sta_ang;
    wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
    wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
    wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
figure (3);
   % [Flag, end_ang] = IKine5DNew(wrobot);   
    [Flag, end_ang] = IKine5D22(wrobot);  
     wrobot.TarJAs =end_ang;
      DrawRobotmo(wrobot,1);
%     if so_option == 2
%         diff_eng =360-abs(end_ang(3) - sta_ang(3));
%         if end_ang(3) - sta_ang(3)>0
%             end_ang(3) = sta_ang(3) - diff_eng;
%         else
%              end_ang(3) = sta_ang(3) + diff_eng;
%         end
%        
%     end
%end_ang(3) =105.82;
 
   for i = 1:num_msur    %壁面绘图
   pp = patch(m_sur{i}(:,1)*1000,m_sur{i}(:,2)*1000,m_sur{i}(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.3,'LineWidth',1);
   end
     
    hold on 
  if num_ob>=1          %判断障碍物是否存在
     
 obp_tran(obs_p);      %障碍物绘图

 end
   %  wrobot.TarJAs =sta_ang;
   %   DrawRobotmo(wrobot,1);
 %   wrobot.TarJAs =end_ang;
    %  DrawRobotmo(wrobot,1);
  
     for i = 1:5             %先让机器人上升5cm,并对起始点和目标点各提升一段距离用作平缓靠近壁面 
        wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
        wrobot.TarGraspMtx(1:3,1:3) = sta_to_bas_R;
    wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
    wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
        if i > 1 
            wrobot.CurJAs = joint_ang(i-1,:);
        end
        [Flag, joint_ang(i,:)] = IKine5DNew(wrobot);
        if Flag == 0
           h=msgbox('起始位姿有误','warn'); 
        uiwait(h,2);
        break;
        end
     end 
      stp = stp + Vn*0.05;
          
      plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);
      plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);
      axis equal
% 
  % obs2 = [-0.6 -0.5 0.3;-0.5 -0.5 0.3;-0.5 -0.2 0.3;-0.6 -0.2 0.3;-0.6 -0.5 0.5;-0.5 -0.5 0.5;-0.5 -0.2 0.5;-0.6 -0.2 0.5];
   %obs_p = [-0.6 -0.5 0;-0.5 -0.5 0;-0.5 -0.2 0;-0.6 -0.2 0;-0.6 -0.5 0.2;-0.5 -0.5 0.2;-0.5 -0.2 0.2;-0.6 -0.2 0.2];
   %obs_p = [-0.65 0.5 0;-0.55 0.5 0;-0.55 0.2 0;-0.65 0.2 0;-0.65 0.5 0.2;-0.55 0.5 0.2;-0.55 0.2 0.2;-0.65 0.2 0.2];
%obs_p = [0.2 0.2 0.37;0.25 0.2 0.37;0.25 0.3 0.37;0.2 0.3 0.37;0.2 0.2 0.5;0.25 0.2 0.5;0.25 0.3 0.5;0.2 0.3 0.5];
%   %[obs2] = enlarge_border(obs_border) ;
% %   obp_tran(obs_border);

%  obp_tran(obs_p);
%  wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人到达目标点的关节角
%  [Flag, end_ang] = IKine5DNew(wrobot);

%  for i = 1:8
%       poly_motion = [p_s;bap];
%        prop_obs = point_pro_sur(obs2(i,:),map_border);      %障碍物点在平面上的投影
%        prop_obs = prop_obs + Vn*0.05;
%       state = insidepoly3(prop_obs,poly_motion);        %判断点是否在空间多边形内部
%       if state == 1  
%       if norm(prop_obs - bap) < 0.4 && dot(obs2(i,:),Vn) <= 0.35
%             h=msgbox('障碍物太近，关节角求解失败','warn'); 
%           uiwait(h,2);
%           return;
%       end
%       else
%           continue;
%       end
%  end

   num_ang = 5;   
while norm(joint_ang(num_ang,:)-end_ang)>1e-2
     [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
     plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
    joi_diffe = abs(end_ang - joint_ang(num_ang,:));   %每个关节角之间的差值足够小，直接到达目标点
    if joi_diffe(1:4)< 3
        num_ang = num_ang +1;
        joint_ang(num_ang,:) = end_ang;
        break;
    end
    dis1 = 10000;
    dis2 = 10000;
    vec_lin = zeros(1,3);
    vec_eff = zeros(1,3);
    
    for j = 1: 4   %求四个关节点
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang(num_ang,:) ,j);       
    end 
    
    joi_p = joi_p/1000;
    joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
        eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,10);
        eff_p2 = cir_seem_poly( joi_lap,joi_p(4,:)-joi_p(3,:),0.2,10);
     
%       
%        if norm(joi_p(4,:))<0.38  %超出机器人工作空间范围
% %              if so_option == 1
% %          
% %                   [ joint_ang] = colli_avoidance22( M_sur,stp_o,enp_o,bap_o,obs_p2,2);
% %                   return;
% %               else
%             h=msgbox('距离基座过近，关节角求解失败','warn'); 
%           uiwait(h,2);
%             break;
%            %  end
%        end
        if  norm(joi_p(4,:)-enp)<0.1           %靠近壁面避碰策略失效时可用
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
       num_inter = floor(max_diffe/2);
       joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        num_ang = num_ang + 1; 
        continue;
       end
      for i= 1: num_msur
         for j=1:3            %求机器人本体连杆距离壁面的最小距离
             if j == 3
                 [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],m_sur{i},4);
             else
             [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],m_sur{i},4);
             end
         if dist < dis1
         dis1 = dist;
         vec_lin = po1 - po2;
         end
         end 
      end
      if num_ob >0
          for i = 1:num_ob
      for j = 1:3       %求机器人本体连杆到障碍物的最小距离
          if j == 3
              [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],obs_p(i*8-7:i*8,:),2);
          else
           [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p(i*8-7:i*8,:),2);
          end
         if dist < dis1
            dis1 = dist;
            vec_lin = po1 - po2;
         end
      end
          end
      
      end
      dis1 = dis1 - 0.05;
if num_ob >0
     for i = 1:num_ob 
       
            [ dist,po1,po2] = dis_compute(joi_p(4,:),obs_p(i*8-7:i*8,:),1);
            dist = dist - 0.2;
         if dist < dis2
             dis2 = dist;
              vec_eff = po1 - po2;
         end
     end
end
         
      for i= 1: num_msur
          [ dist,po1,po2] = dis_compute(eff_p,m_sur{i},5);
         if dist < dis2 
         dis2 = dist ;
         vec_eff = po1 - po2;
         end
      end 
      [state(num_ang-4),vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,joi_p(4,:),enp);
      if state(num_ang-4) == 0
%           if so_option == 1
%            %   break;
%               [ joint_ang] = colli_avoidance22( M_sur,stp_o,enp_o,bap_o,obs_p2,2);
%               return;
%           else
           h=msgbox('发生碰撞，规划失败','warn'); 
        uiwait(h,2);
        break;
        %  end
      elseif state(num_ang-4) == 1
           [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
       num_inter = floor(max_diffe/3);
       joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        num_ang = num_ang + 1; 
      elseif state(num_ang-4) == 2 
           %vec_repul = vec_eff/norm(vec_eff);    
         next_p = joi_p(4,:) + vec_re * 0.01;
         wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
        wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
        wrobot.CurJAs=joint_ang(num_ang,:);
         [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(wrobot);
         if Flag == 0
             num_up = num_up +1;
             if num_up == 10
%               if so_option == 1              
%                   so = 2;
%                   [ joint_ang] = colli_avoidance22( M_sur,stp_o,enp_o,bap_o,obs_p2,so);
%                   return;
%               else
                     h=msgbox('避碰规划失败','warn'); 
                     uiwait(h,2);
            break;
           %   end
             else
                joint_ang(num_ang+1,:) =  joint_ang(num_ang,:) +[0 2 0 0 0];
             end
         end
       num_ang = num_ang + 1;
     
      end
      %joi_p(4,:)
%    if dis2 >= dis_warn - 0.002           %距离大于需要注意的距离
%        [max_diffe,num_j]=max(abs(end_ang-joint_ang(num_ang,:)));
%        num_inter = floor(max_diffe/3);
%        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
%         num_ang = num_ang + 1; 
%    end
%    if dis2 < dis_warn - 0.002 && dis2 > dis_safe   %距离大于安全距离但在需要注意的距离内
%        %joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*2,0,0,0,0];
%         num_inter = floor(max_diffe/1);
%        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
%        num_ang = num_ang +1;  
%    end
%     if dis2 <= 0
%         h=msgbox('发生碰撞，规划失败','warn'); 
%         uiwait(h,2);
%         break;
%    end
%    if dis2 <= dis_safe    %距离小于安全距离
%          vec_repul = vec_eff/norm(vec_eff);
%          next_p = joi_p(4,:) + vec_repul * 0.01;
%          wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
%         wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
%          [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(wrobot);
%          if Flag == 0
%                  h=msgbox('避碰规划失败','warn'); 
%         uiwait(h,2);
%         break;
%          end
%        num_ang = num_ang + 1;
%    end

    if num_ang>200
        
%          if so_option == 1
%           
%                   so = 2;
%                   [ joint_ang] = colli_avoidance22( M_sur,stp_o,enp_o,bap_o,obs_p2,so);
%                   return;
%               else
                     h=msgbox('规划次数内无结果','warn'); 
                     uiwait(h,2);
            break;
        %  end
    end
end
% 
 for i = 1:5               %使机器人末端点往Z轴方向靠近5cm
      [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
      end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.01*1000;
     
      wrobot.TarGraspMtx(1:3,4) = end_p';
      wrobot.CurJAs=joint_ang(num_ang+i-1,:);
       wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
    wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
    wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
       [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(wrobot);
 end
      num_ang = num_ang + 5;
      joint_ang = [sta_ang;joint_ang];
 num_joi = size(joint_ang,1);
  wrobot.TarJAs =joint_ang(6,:);
  DrawRobotmo(wrobot,1);
   wrobot.TarJAs =joint_ang(num_joi,:);
   DrawRobotmo(wrobot,1);
 end_ang
  for i = 1: num_joi
      [ T,joi_po(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
  end
      plot3(joi_po(:,1), joi_po(:,2),joi_po(:,3),'-k','LineWidth',2);
      axis([-2000,2000,-2000,2000,0,2000])
end


function[obs_sur] =obp_tran(obp)%将代表障碍物的8个三维空间点转化为障碍物元素（5个面元素和8个线段元素）
num_ob = size(obp,1)/8;
obs_sur = zeros(5,4*3,num_ob);
obs_line = zeros(8,2*3,num_ob);
for i = 1:num_ob
    for j=1:4
        if j==4
           obs_sur(j,:,i)=[obp(8*i-4,:),obp(8*i-7,:),obp(8*i-3,:),obp(8*i,:)]; 
        else
            obs_sur(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:),obp(8*i-3+j,:),obp(8*i-4+j,:)];
        end  
    end
    obs_sur(5,:,i)=[obp(8*i-3,:),obp(8*i-2,:),obp(8*i-1,:),obp(8*i,:)];
end
for i=1:num_ob
    for j=1:4
        obs_line(j,:,i)=[obp(8*i-8+j,:),obp(8*i-4+j,:)];
    end
    for j=5:8
        if j==8
            obs_line(j,:,i)=[obp(8*i,:),obp(8*i-3,:)];
        else
            obs_line(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:)];
        end
    end
end
% for i=1:5
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%           plot3(sur_x*1000, sur_y*1000,sur_z*1000,'-g','LineWidth',2);
for i = 1: num_ob 
 fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
 patch('Faces',fac_obs,'Vertices',obp(i*8-7:i*8,:)*1000,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',0.3);
         hold on;
end
 

end
function[obs_bor] = enlarge_border(obs)   %将障碍物的边界扩大
r = 0.2;
for i = 1:2
    obs_bor(i*4-3,:) = obs(i*4-3,:) + (obs(i*4-3,:)-obs(i*4,:))/norm(obs(i*4-3,:)-obs(i*4,:))*r + (obs(i*4-3,:)-obs(i*4-2,:))/norm(obs(i*4-3,:)-obs(i*4-2,:))*r;
    obs_bor(i*4,:) = obs(i*4,:) + (obs(i*4,:)-obs(i*4-3,:))/norm(obs(i*4,:)-obs(i*4-3,:))*r + (obs(i*4,:)-obs(i*4-1,:))/norm(obs(i*4,:)-obs(i*4-1,:))*r;
    for j = 1:2
        obs_bor(i*4-3+j,:) = obs(i*4-3+j,:) + (obs(i*4-3+j,:)-obs(i*4-2+j,:))/norm(obs(i*4-3+j,:)-obs(i*4-2+j,:))*r + (obs(i*4-3+j,:)-obs(i*4-4+j,:))/norm(obs(i*4-3+j,:)-obs(i*4-4+j,:))*r;
    end
end
end

function [prop] = point_pro_sur(point,poly)      %点在平面上的投影
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
vec = vec/norm(vec);
prop = point-vec*dot(point-poly(1,:),vec);
end

function[vec_repul] = repul_veccompute(obs,map_sur,joi_p)   %计算障碍物对机器人整体的影响，取障碍物中心点作为作用点，求其对机器人每一个关节点的作用向量再对所有作用向量叠加,输入障碍物点和机器人关节点
   obs_aver = zeros(1,3);
   vec_p = zeros(size(joi_p,1),3);
   vec_repul = zeros(1,3);
   %robot_sur = [joi_p(1,:);joi_p(2,:);joi_p(3,:)];
 
   for i= 1:size(obs,1)
       obs_aver = obs_aver + obs(i,:);
   end
   obs_aver = obs_aver/size(obs,1);
    pro_obs = point_pro_sur(obs_aver,map_sur);      %用障碍物，基座，障碍物在壁面的投影点组成参考平面
    obs_sur = [obs_aver;pro_obs;[0 0 0]];
    for i= 1:4
        joi_pro(i,:) = point_pro_sur(joi_p(i,:),obs_sur);
    end
   
   for i = 1:4
       vec_p(i,:) = (joi_pro(i,:) - obs_aver)/norm(joi_pro(i,:) - obs_aver);
       vec_repul = vec_repul + vec_p(i,:);
   end
      vec_repul = vec_repul + (-obs_aver)/norm(obs_aver); 
      vec_repul = vec_repul/norm(vec_repul);
end

function [state] = insidepoly3(point,poly)        %判断点是否在空间多边形内部
num_p=size(poly,1);
mean_p=zeros(1,3);
kk=0;
for i=1:num_p
mean_p = mean_p + poly(i,:);
end
mean_p = mean_p/num_p;
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
if abs(dot(point-poly(1,:),vec))>1e-5
    state = 0;
    return;
end
for i=1:num_p
    ls = poly(i,:);
    if i==num_p
    le = poly(1,:);
    else
        le = poly(i+1,:);
    end
    if (dot(cross(point-ls,le-ls),cross(mean_p-ls,le-ls))<0)
         kk=1;
        break;
    end
end
if kk==0
    state =1;
else
    state=0;
end
end

function[state,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,eff_p,enp)   %计算机器人本体斥力向量和机器人末端斥力向量和目标引力对机器人运动的影响，输入本体最小距离和末端最小距离，当前末端点和目标点
   vec_re = zeros(1,3);
    dis_warn = 0.05;  %注意距离为5cm
  dis_safe = 0.02;  %最小安全距离为2cm
  vec_lin = vec_lin/norm(vec_lin);
  vec_eff = vec_eff/norm(vec_eff);
  vec_tar = (enp - eff_p)/norm(enp - eff_p);
     if dis1 <= 0.001 || dis2 <= 0.001
         state = 0 ;  %发生碰撞
         return;
     end
     if dis1 >= dis_warn && dis2 >= dis_warn
         state = 1;  %没有发生碰撞
         return;
     end
     if dis1 <dis_warn && dis1 >=dis_safe
           rec_lin =(  0.1*(1/dis1-1/0.05))* vec_lin;
     elseif dis1 >0 && dis1 < dis_safe
         rec_lin = 3 * vec_lin;
     elseif dis1 >= dis_warn
         rec_lin = 0;
     end
     if dis2 <dis_warn && dis2 >= dis_safe
           rec_eff =(  0.1*(1/dis2-1/0.05))* vec_eff;
     elseif dis2 >0&& dis2 < dis_safe
         rec_eff = 3 * vec_eff;
     elseif dis2 >= dis_warn
         rec_eff = 0;
     end
     state = 2;
      vec_re =  vec_tar + rec_lin + rec_eff ;
     % vec_re = rec_lin + rec_eff ;
      vec_re = vec_re/norm(vec_re);
end
function [avp] = average_p(obp)%求障碍物中心点
avp = zeros(1,3);
for i = 1:size(obp,1)
    avp = avp + obp(i,:);
end
   avp = avp/size(obp,1); 
end