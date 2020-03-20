function [lastbas_R,laststp_R,lastjoi,joint_ang ] = colli_avoidance( M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option)

%  求解壁面单步无碰运动
%[ joint_ang,p_s ] = joint_angle(stp,enp,bap);
global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;
[BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();
CurHMatrix  = eye(4);
enp_o = enp;
stp_o = stp;
bap_o = bap;
bap1 = bap;
wrobot = struct('DoFs',5,'LinksLen',[335,293.2,293.2,335],'ModuleRadii',50,...
'CurGraspMtx',CurHMatrix,'CurJAs',[0 8.4728 -16.9456 8.4728 0],'FixedGripper',1,'TarGraspMtx',eye(4,4),...
'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
 wrobot.CurJAs = fir_joi;
dis_warn = 0.04;  %注意距离为4cm
dis_safe = 0.02;  %最小安全距离为2cm
dis = 10000;
TO_SURFACE_DISTANCE = 0.05;%单步规划时，吸盘抬起的距离
vec_lin = zeros(1,3);
vec_eff = zeros(1,3);
num_ob = floor(size(obs_p,1)/8);
wrobot.CurGraspMtx(1:3,4) = bap*1000;
wrobot.CurGraspMtx(1:3,1:3) = bas_R;
figure (3);
  pp = patch(M_sur(:,1)*1000,M_sur(:,2)*1000,M_sur(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.5,'LineWidth',1);%壁面绘图
  %画图范围设置 by:CH
  set(gca,'xlim',[-100,1500])%将x轴上的取值范围设置为[-100,1500] 
  xlabel('x');
  set(gca,'ylim',[-1500,1500])%将y轴上的取值范围设置为[-1500,1500] 
  ylabel('y');
  set(gca,'zlim',[200,1500])%将z轴上的取值范围设置为[200,1500] 
  zlabel('z');
  hold on 
   
   if num_ob>=1          %判断障碍物是否存在
      obp_tran(obs_p);      %障碍物绘图
     for i = 1:num_ob
              obs_p2(i*8-7:i*8,:) = obs_p(i*8-7:i*8,:);
     end
   end 

plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);%画起点
plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);%画终点
axis equal

num_mp = size(M_sur,1);
Vnm=cross(M_sur(2,:)-M_sur(1,:),M_sur(3,:)-M_sur(2,:));   %计算多边形法向量
Vn_norm=norm(Vnm);
Vnm=Vnm/Vn_norm;
bas_x = (M_sur(2,:)-M_sur(1,:))/norm(M_sur(2,:)-M_sur(1,:));   %求机器人运动的基坐标系
bas_z = Vnm(1,:); 
bas_y = cross(bas_z,bas_x);
% bas_R = [bas_x',bas_y',bas_z'];
 wor_to_bas_R = bas_R';   %世界坐标系在基座标系的表示
 wtoba_p = (-wor_to_bas_R*bap')';
 Vn = (wor_to_bas_R*Vnm')';
 
for j = 1:num_mp
    m_sur(j,:) = (wor_to_bas_R*M_sur(j,:)')' + wtoba_p; 
end

 enp = (wor_to_bas_R*enp')'+ wtoba_p;   %目标点在基座标系下的表示
 stp = (wor_to_bas_R*stp')'+ wtoba_p;   %起始落足点在基座标下的表示
 enp = enp + Vn*TO_SURFACE_DISTANCE;    %目标吸附点的辅助路径径点
 if num_ob>=1                            %判断障碍物是否存在，若存在，转换至基座坐标系下
 for i = 1:size(obs_p2,1)
     obs_p2(i,:) = (wor_to_bas_R*obs_p(i,:)')'+ wtoba_p;
 end
 %obs_p2 = obs_p;
 end
%      wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
%      wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向  

%//////////////////////////////////////////求初始、目标关节角
wrobot.TarGraspMtx(1:3,1:3) = stp_R;
wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
[Flag,sta_ang] = IKine5DNew(wrobot);                %求机器人起始构型的关节角
un_sta = sta_ang(1,5);                               %机器人不变的旋转关节（非基座）值  
wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人目标构型的关节角
wrobot.CurJAs = sta_ang;
[Flag, end_ang] = IKine5DNew(wrobot); 
% %***********************2020.3.15 修改:加入对目标关节变量第三个关节的判断
% if end_ang(1,3) > 0
%     end_ang(1,3) = -end_ang(1,3);
%     end_ang(1,2) = -end_ang(1,2);
%     end_ang(1,4) = -end_ang(1,4);
% end
% %//////////////////////////////////////////end

%/////////////////////%I关节换一个方向旋转
if so_option == 2
    diff_eng =360-abs(end_ang(1) - sta_ang(1));
    if end_ang(1) - sta_ang(1)>0
        end_ang(1) = sta_ang(1) - diff_eng;
    else
         end_ang(1) = sta_ang(1) + diff_eng;
    end     
       Flag
end
%*********************end

wrobot.TarJAs =sta_ang;
 % DrawRobotmo(wrobot,1);
wrobot.TarJAs =end_ang;
%  DrawRobotmo(wrobot,1);
  end_ang
%%求解起始点目标点在同一壁面的情形
%//////////////////////////////// 1）先让机器人上升5cm,对起始点和目标点各提升一段距离用作平缓靠近壁面 
%2020.2.27改：将这个上升距离参数改成全局，方便修改
 for i = 1:TO_SURFACE_DISTANCE * 100             
    wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
    if i > 1 
        wrobot.CurJAs = joint_ang(i-1,:);
    end
    %@@@@@@@@@@@@@@@@@@@@@@@IKine5DNew返回的是不是角度增量 answer：不是，返回的是目标角度
    [Flag, joint_ang(i,:)] = IKine5DNew(wrobot);%Flag是逆运动学是否有解的标志，joint_ang(i,:)是各个关节的角度
 end
 stp = stp + Vn * TO_SURFACE_DISTANCE;   
 bap =  Vn * TO_SURFACE_DISTANCE;
%//////////////////////////////// end


num_ang = 5;
i = 0;
%///////////////////////////////////////////第一个while
while norm(joint_ang(num_ang,:)-end_ang)>1e-4%如果上升5cm的最后一步跟目标姿态足够近似的话就不进入这个循环
    
      [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);%joint_ang(num_ang,:)是最后一帧的角度；这里的joi_pp是基座坐标系下的
      joi_pp = (bas_R*joi_pp')'+ bap1*1000;%joi_pp转到世界坐标系下
      plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);%粉红色圆圈，吸盘中心
      joi_diffe = abs(end_ang - joint_ang(num_ang,:));   %每个关节角之间的差值足够小，直接到达目标点
    
    %////////////除了夹持器，每个角度差小于3°的话
    if joi_diffe(1:4)< 3
    num_ang = num_ang +1;
    joint_ang(num_ang,:) = end_ang;
    break;
    end
    %*****************************end
      
    %////////////如果超过规定步数,将step_type换成2求解
    if num_ang>300
    if so_option == 1%如果是壁面规划
       %   break;
          %@@@@@@@@@@@@@@@@为什么如果超过规定步数就要将step_type换成2？
          [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance( ...
              M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
          return;
    else
         h=msgbox('规划次数内无结果','warn'); 
         uiwait(h,2);
          break;
    end
    end
    %*****************************end
     
     %////////////如果障碍物不存在，则按最大角度差的比例，增加一个角度增量
    if num_ob <1    
      [max_diffe,num_j]=max(abs(end_ang-joint_ang(num_ang,:)));
      num_inter = floor(max_diffe/3);
      joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
      num_ang = num_ang + 1; 
      continue;
    end
    %*****************************end
    
      dis1 = ones(1,num_ob)*10000;
      dis2 = ones(1,num_ob)*10000;
      
    %////////////%靠近壁面避碰策略失效时可用，该距离可斟酌修改（用末端的xy减去目标xy）
    for j = 1: 4   %求四个关节点
        [ T,joi_p(j,:)] =  Kine5D(joint_ang(num_ang,:) ,j);
    end 
    joi_p = joi_p/1000;
    %这里不大懂
    if  norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0])<0.08           
       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
       num_inter = floor(max_diffe/3);
       joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
       num_ang = num_ang + 1; 
        continue;
    end
    %*****************************end
    
   %////////////%末端距离基座过近，则将step_type换成2求解
   %if norm(joi_p(4,:))>0.586 || norm(joi_p(4,:))<0.38  %超出机器人工作空间范围
    if norm(joi_p(4,:))<0.38  %末端距离基座过近
        if so_option == 1
          [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
          return;
    else
          h=msgbox('末端过近，关节角求解失败','warn'); 
          uiwait(h,2);
          break;
        end
    end
    %*****************************end

    %////////////%对所有障碍：以下分别计算吸盘与障碍的距离以及本体对障碍的距离，并在两者中取最小（dis）
    for k = 1:num_ob
            if insidepoly3(obs_p2(8*k-7,:),m_sur) ==1%如果当前障碍在这个壁面内
                 eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);%获取吸盘圆形平面
       % [ dis1(k),po1,po2] = dis_compute(joi_p(4,:),obs_p2(k*8-7:k*8,:),1);   %用机器人末端点去对壁面障碍作测试
      [ dis1(k),po1,po2] = dis_compute(eff_p,obs_p2(k*8-7:k*8,:),6); 
             else
                continue;
            end    
    end
    %对所有障碍：
    for k = 1:num_ob
        if insidepoly3(obs_p2(8*k-7,:),m_sur) == 0
             for j=1:3            %求机器人本体连杆距离障碍物的最小距离
                 [ dist(j),po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p2(i*8-7:i*8,:),2);
                   dis2(k) = min(dist);
             end
        else
            continue;
        end
    end
     dis2 = dis2 - TO_SURFACE_DISTANCE;
     [mind,min_ob2] =min(dis2);
     [mind,min_ob1] =min(dis1);
    dis = min([min(dis1),min(dis2)]);
    %*****************************end
    
    %////////////%如果距离大于需要注意的距离，则按正常策略进行关节角度规划
    if dis >=dis_warn              
       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
       num_inter = floor(max_diffe/3);
       joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        num_ang = num_ang + 1;
        continue;
    end
    %*****************************end
    
    %////////////%已发生碰撞，则将step_type换成2 @@@@@@@@@@@@@@@@@@@@@为什么要换成2
    if dis <= 0.003       
        if so_option == 1
              [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
              return;
        else
              h=msgbox('发生碰撞，规划失败','warn'); 
              uiwait(h,2);
              break;
        end
    end
   %*****************************end
   
   %////////////%距离（吸盘更靠近障碍物）大于安全距离但在需要注意的距离内 
   %2020.2.27跑main的时候没有进入过这个if
   %问一问师兄这是什么意思
   if dis <dis_warn && dis>dis_safe && min(dis1)<min(dis2) 
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + ...
            [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
        %基座i关节旋转3°
        if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 ||...
                abs(end_ang(1)-joint_ang(num_ang+1,1))<=2
            
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
             num_inter = floor(max_diffe/1.5);   %该参数可斟酌修改
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        elseif  joi_p(4,3)-obs_p2(min_ob1*8-3,3)<0.02 %为什么是 obs_p2(min_ob1*8-3,3) 
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 3 0 -3 0];
        end
              num_ang = num_ang +1;
              continue;
   end
   %*****************************end
   
    %////////////%距离小于安全距离,判断障碍物上升或者下降是否带来与障碍物距离的缩小，再以此调整主运动方向
   if dis <= dis_safe  && min(dis1)<min(dis2)   %如果末端离障碍物更近
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 3 0 -2 0];%向基座缩一点
        [ dis1kt,po1,po2] = dis_compute(joi_p(4,:),obs_p2(min_ob1*8-7:min_ob1*8,:),1);%用机器人末端点去对最近障碍作测试
        pok = (po1-po2)/norm(po1-po2);
        joi_ps = point_pro_sur(joi_p(4,:),M_sur);
        
         %////////////%判断障碍物对末端点的影响和主运动的方向是否相同（以吸盘在壁面的投影点和基座之间的连线为分界，判断障碍物与终点是否位于同一侧）
         if dot(cross(joi_ps-bap,po1 - po2),cross(joi_ps-bap,enp-Vn*TO_SURFACE_DISTANCE-bap))>0   
             joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                 [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*(1+(dis_safe-dis)*60),0,2,0,0];%如果位于同一侧
         else
             joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                 [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-(1+(dis_safe-dis)*60),0,2,0,0];%如果不位于同一侧
         end
         %*****************************end
         
         %////////////%避免主运动方向偏离过大 @@@@@@@@@为什么会出现这种情况
         if (end_ang(1)-joint_ang(num_ang-2,1))*(end_ang(1)-joint_ang(num_ang-3,1))<0 
             if  norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0])>=0.1    
                 vec_re =( [enp(1),enp(2),0]-[joi_p(4,1),joi_p(4,2),0])/norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0]);
                 next_p = joi_p(4,:) + vec_re * 0.01;
                 wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                 wrobot.TarGraspMtx(1:3,1:3) = T(1:3,1:3);
                 wrobot.CurJAs=joint_ang(num_ang,:);
                 [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(wrobot);
             if  Flag == 0
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 -1 2 -1 0];
             end
             else
                   [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                   num_inter = floor(max_diffe/2);
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
             end
             num_ang = num_ang + 1;
             continue;
         end
         %*****************************end
         
          %////////////%@@@@@@@@@这里看不懂在干嘛
         joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
             [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*2,0,0,0,0];
         for j = 1: 4   %求四个关节点
            [ T,joi_p2(j,:)] =  Kine5D(joint_ang(num_ang+1,:) ,j);
         end 
         joi_p2 = joi_p2/1000;
         eff_p2 = cir_seem_poly( joi_p2(4,:),joi_p2(4,:)-joi_p2(3,:),0.2,20);
           %用机器人末端点去对壁面障碍作测试     
         [ dis1kt,po1,po2] = dis_compute(eff_p2,obs_p2(min_ob1*8-7:min_ob1*8,:),6);
%          poo = [po1;po2];
%          plot3(poo(:,1),poo(:,2),poo(:,3),'-r');
         vec_f = dot(pok,Vn);
         if abs(vec_f)<0.01 && dis1kt <dis
            joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-5,0,0,0,0];
            num_ang = num_ang + 1;
            continue;
         end
         if dis1kt <dis  || num_ang>10 
             joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                 [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-2,0,0,0,0];
             num_ang = num_ang + 1;
             continue;
         end      
         num_ang = num_ang + 1;
         continue;
   end
   %*****************************end
   
        %%空间障碍物部分
   if dis <dis_warn && dis>dis_safe && min(dis2)<min(dis1) %距离大于安全距离但在需要注意的距离内
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
        %/////////////////@@@@@@@@@@@为什么用这个判断依据？？
        if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 || abs(end_ang(1)-joint_ang(num_ang+1,1))<=2 %基座I关节运动到goal_angle+-3°范围
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
             num_inter = floor(max_diffe/3);
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        end
       num_ang = num_ang +1; 
       continue;
   end
   
     %////////////%距离小于安全距离（本体更近）
     if dis <= dis_safe  && min(dis2)<min(dis1)   
      [vec_repul] = repul_veccompute(obs_p2(min_ob2*8-7:min_ob2*8,:),m_sur,joi_p);%障碍物对机器人整体的影响
      effector_p = point_pro_sur(joi_p(4,:),m_sur);
      vec_eff = effector_p/norm(effector_p);
        if dot(vec_repul,Vn)<-0.7 % 大于135°%@@@@@@@@@@@@@@@@为什么要这样判断？？
            if joint_ang(num_ang,2)<-30 || joint_ang(num_ang,4) < -30
                 joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 -0.5 -1 3 0];
            else
                 joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 -1 2 -1 0];
            end
                       
        elseif dot(vec_repul,vec_eff)<-0.7 % 大于135°%@@@@@@@@@@@@@@@@为什么要这样判断？？
            joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 1 -2 1 0];    
        end
         if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 ||...
                 abs(end_ang(1)-joint_ang(num_ang+1,1))<=3
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
             num_inter = floor(max_diffe/3);
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        end
       num_ang = num_ang + 1;
       continue;
     end
     %*****************************end
     
end
 %////////////%使机器人末端点往Z轴方向靠近TO_SURFACE_DISTANCE * 100cm
 for i = 1:TO_SURFACE_DISTANCE * 100              
      [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
      end_p = end_p - Vn*(0.01)*1000;
      wrobot.TarGraspMtx(1:3,1:3) = T(1:3,1:3);
%        T(1:3,2) = T(1:3,2)*-1;
%        T(1:3,3) = T(1:3,3)*-1; 
      wrobot.TarGraspMtx(1:3,4) = end_p';
      wrobot.CurJAs=joint_ang(num_ang+i-1,:);
       [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(wrobot);
       if Flag == 0 
            h=msgbox('下降过程无解','warn'); 
        uiwait(h,2);
        break;
       end
 end
num_ang = num_ang + 6;
joint_ang = [sta_ang;joint_ang];
num_joi = size(joint_ang,1);
joint_ang(:,5) = un_sta;
 %*****************************end
 

 
  wrobot.TarJAs =joint_ang(6,:);
  DrawRobotmo(wrobot,1);%画起点处机器人抬起的模型
   wrobot.TarJAs =joint_ang(num_ang-6,:);
   DrawRobotmo(wrobot,1);%画终点处机器人抬起的模型
   aa = abs(floor((end_ang(1)-sta_ang(1))/4));

num_pro = num_joi - 10;                  %%对路径做平滑处理
 num_pro = floor(num_pro/2);
 for i = 1: num_pro
    dis1 = ones(1,num_ob)*10000;
    dis2 = ones(1,num_ob)*10000;
    joint_ang2 = (joint_ang(i*2-1+5,:)+joint_ang(i*2+1+5,:))/2;   
    if norm(joint_ang(i*2+5,:)-joint_ang2)<0.01
        continue;
    end
    for j = 1: 4   %求四个关节点
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
    end 
    
    joi_p = joi_p/1000;
    
 for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==1
        eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);
   % [ dis1(k),po1,po2] = dis_compute(joi_p(4,:),obs_p2(k*8-7:k*8,:),1);   %用机器人末端点去对壁面障碍作测试
  [ dis1(k),po1,po2] = dis_compute(eff_p,obs_p2(k*8-7:k*8,:),6); 
         else
            continue;
        end    
 end
    for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==0
     for j=1:3            %求机器人本体连杆距离障碍物的最小距离
         [ dist(j),po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p2(k*8-7:k*8,:),2);
           dis2(k) = min(dist);
     end
        else
            continue;
        end
    end
     dis2 = dis2 - TO_SURFACE_DISTANCE;
     [mind,min_ob2] =min(dis2);
     [mind,min_ob1] =min(dis1);
    dis = min([min(dis1),min(dis2)]);
     if dis >= 0.005                %已发生碰撞
         joint_ang(2*i+5,:) = joint_ang2;
      else
          continue;   
    end

 end
 for i = 1: num_pro-1
    dis1 = ones(1,num_ob)*10000;
    dis2 = ones(1,num_ob)*10000;
    joint_ang2 = (joint_ang(i*2+5,:)+joint_ang(i*2+2+5,:))/2;   
    if norm(joint_ang(i*2+6,:)-joint_ang2)<0.01
        continue;
    end
    for j = 1: 4   %求四个关节点
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
    end 
    
    joi_p = joi_p/1000;
    
 for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==1
        eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);
   % [ dis1(k),po1,po2] = dis_compute(joi_p(4,:),obs_p2(k*8-7:k*8,:),1);   %用机器人末端点去对壁面障碍作测试
  [ dis1(k),po1,po2] = dis_compute(eff_p,obs_p2(k*8-7:k*8,:),6); 
         else
            continue;
        end    
 end
    for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==0
     for j=1:3            %求机器人本体连杆距离障碍物的最小距离
         [ dist(j),po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p2(k*8-7:k*8,:),2);
           dis2(k) = min(dist);
     end
        else
            continue;
        end
    end
     dis2 = dis2 - TO_SURFACE_DISTANCE;
     [mind,min_ob2] =min(dis2);
     [mind,min_ob1] =min(dis1);
    dis = min([min(dis1),min(dis2)]);
     if dis >= 0.005                %已发生碰撞
         joint_ang(2*i+6,:) = joint_ang2;
      else
          continue;   
     end
 end
 
  for i = 1: num_joi
      [ T,joi_p(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
      joi_p(i,:) = (bas_R*joi_p(i,:)')'+bap1*1000;
  end
 
   T(1:3,2) = T(1:3,2)*-1;
   T(1:3,3) = T(1:3,3)*-1;
  lastbas_R = bas_R*T(1:3,1:3);
   laststp_R = T(1:3,1:3)';
 % laststp_R = bas_R*lastbas_R';
   laststp_R(1:3,2) =  laststp_R(1:3,2)*-1;
   laststp_R(1:3,3) =  laststp_R(1:3,3)*-1;
   lastjoi=[0,joint_ang(num_ang,4),joint_ang(num_ang,3),joint_ang(num_ang,2),0];%将关节角顺序倒转，为下一步作准备
  plot3(joi_p(:,1), joi_p(:,2),joi_p(:,3),'-k','LineWidth',2);%用黑线画出末端路径
      
     % axis([-1000,1000,-1000,1000,0,500])
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
for i = 1: num_ob                                              %需要把障碍物画出来时则加上此段
 fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
 patch('Faces',fac_obs,'Vertices',obp(i*8-7:i*8,:)*1000,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',1);
 
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
   
   for i = 1:4%障碍物对四个关节点的作用求和
       vec_p(i,:) = (joi_pro(i,:) - obs_aver)/norm(joi_pro(i,:) - obs_aver);
       vec_repul = vec_repul + vec_p(i,:);
   end
      vec_repul = vec_repul + (-obs_aver)/norm(obs_aver); %障碍物对四个关节点的作用加上障碍物对基座的作用
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
if abs(dot(point-poly(1,:),vec))>1e-3
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
