classdef SingleMotionPlanner < handle
%% 描述
% SingleMotionPlanner定义了一个可以进行5自由度机械臂无碰运动规划的类。
% 需要绘图时，SingleMotionPlanner必须与drawing文件夹一起配合使用
% SingleMotionPlanner用于双足吸盘攀爬机器人的单步无碰攀爬运动规划。
% 注意：该类调用了大批其它相互关联的函数，并不能单独工作。
%% 使用示例
% %1）修改障碍和壁面数据
% 在input_map2函数和surfaces函数中修改
% %2）指定壁面规划落足点和落足点类型
% wallFootStep = [0.4473   -1.4785    0.8075;
%     0.2223   -1.4755    0.4768;
%     0.6933   -1.4767    0.7893;
%     1.2739   -0.7051    0.8308;
%     1.2677   -0.1285    0.7683;
%     1.2807    0.2039    0.4223;
%     1.2603    0.5580    0.6937;
%     0.5825    1.2404    0.6781;
%     0.3618    1.2339    1.0471;
%     0.4494    1.2441    0.6526;]
% stepOption =[1     2     2     1     1     2     2     1     2     2] 
% %3）构造SingleMotionPlanner对象
% planner = SingleMotionPlanner(wallFootStep, stepOption, 0) %不画图
% planner = SingleMotionPlanner(wallFootStep, stepOption, 0) %画图
% %4)单步规划
% planner.motionplan()
properties
    
    %%障碍物数据 
    obstruct_borders = []; %障碍物顶点
    obstruct_height = []; %障碍物高度
    valiable_obstruct_index = []; %有效障碍物引索
    
    %%壁面规划点数据
    wall_planning_points = [];
    step_type = []; %落足点类型
    
    %%壁面数据
    surface = []
    num_point_each_surface = []
    num_surfaces = 0;
    surface_step = cell(1,3); %壁面规划的相邻三个点各自所在的平面
    
    %%机器人数据
    link_1 = 0.335;
    link_2 = 0.2932;
    r = 0.1998;
    wrobot = [];
    
    %规划数据
    Ba = [];
    Ba_R = [];
    joint_ang = [];
    
    %%规划设置
    draw_flag = 0; %1:画图； 0不画图
end
    methods
       
        function self = SingleMotionPlanner(wall_planning_points, step_type, draw_flag)
            [~, self.obstruct_borders, self.obstruct_height] = input_map2;
            [self.surface, self.num_point_each_surface, self.num_surfaces, ~, ~] = surfaces;
            self.step_type = step_type;
            self.wall_planning_points = wall_planning_points;
            self.draw_flag = draw_flag;
            CurHMatrix  = eye(4);
            self.wrobot = struct('DoFs',5,'LinksLen',[335,293.2,293.2,335],'ModuleRadii',50,...
            'CurGraspMtx',CurHMatrix,'CurJAs',[0 8.4728 -16.9456 8.4728 0],'FixedGripper',1,'TarGraspMtx',eye(4,4),...
            'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
        end
        
        function expand_surface = expand_surface(self)
            for i=1:self.num_surfaces
                v1= (self.surface{i}(self.num_point_each_surface(i),:)-self.surface{i}(1,:))/norm(self.surface{i}(self.num_point_each_surface(i),:)-self.surface{i}(1,:));
                v2= (self.surface{i}(2,:)-self.surface{i}(1,:))/norm(self.surface{i}(2,:)-self.surface{i}(1,:));
                ang_cos = dot(v1,v2);
                ang_sin = sqrt(1-ang_cos^2);
                l = self.r/ang_sin;
                expand_surface{i}(1,:) = self.surface{i}(1,:)+ v1*l +v2*l;

                v1=( self.surface{i}(1,:)-self.surface{i}(self.num_point_each_surface(i),:))/norm(self.surface{i}(1,:)-self.surface{i}(self.num_point_each_surface(i),:));
                v2= (self.surface{i}(self.num_point_each_surface(i)-1,:)-self.surface{i}(self.num_point_each_surface(i),:))/norm(self.surface{i}(self.num_point_each_surface(i)-1,:)-self.surface{i}(self.num_point_each_surface(i),:));
                ang_cos = dot(v1,v2);
                ang_sin = sqrt(1-ang_cos^2);
                l = self.r/ang_sin;
                expand_surface{i}(self.num_point_each_surface(i),:) = self.surface{i}(self.num_point_each_surface(i),:)+ v1*l +v2*l;

               for j=2:self.num_point_each_surface(i)-1
                   v1= (self.surface{i}(j+1,:)-self.surface{i}(j,:))/norm(self.surface{i}(j+1,:)-self.surface{i}(j,:));
                   v2= (self.surface{i}(j-1,:)-self.surface{i}(j,:))/norm(self.surface{i}(j-1,:)-self.surface{i}(j,:));
                   ang_cos = dot(v1,v2);
                   ang_sin = sqrt(1-ang_cos^2);
                   l = self.r/ang_sin;
                   expand_surface{i}(j,:) = self.surface{i}(j,:)+ v1*l +v2*l;   
               end
           end
        end
        
        function [ obs_p ] = map_obs(self, i)
            %UNTITLED 此处显示有关此函数的摘要
            % 将第i个障碍物的坐标点转换到壁面坐标系下
            %num_map = size(map_border,1);
            map_border = self.expand_surface{i};
            obs_border = self.obstruct_borders{i};
            obs_height = self.obstruct_height{i};         
            
            num_obs = floor(size(obs_border,1)/4); %障碍的表示方式是障碍物矩形加高度
            Vz = cross(map_border(2,:)-map_border(1,:),map_border(3,:)-map_border(2,:));   %计算多边形法向量
            Vz = Vz/norm(Vz);

            Vx = (map_border(2,:)-map_border(1,:))/norm(map_border(2,:)-map_border(1,:));   %求壁面坐标系在世界坐标系下的表示
            Vy =  cross(Vz,Vx);

            stw_R = [Vx',Vy',Vz'];

            obs_p = [];
            %将障碍物转成obs(pointx1,pointy1, pointx2, pointy2, height1...)的格式
            for i = 1:num_obs
                obs_p(i*8-7:i*8-4,1:2) = obs_border(i*4-3:i*4,:);
                obs_p(i*8-3:i*8,1:2) = obs_border(i*4-3:i*4,:);
                obs_p(i*8-3:i*8,3) = obs_height(i);
            end
            %将障碍物坐标点转换到壁面坐标系下
            for i = 1:num_obs
                for j = 1:4
                    obs_p(i*8-8+j,:) = (stw_R*obs_p(i*8-8+j,:)'+ map_border(1,:)')' ;
                    obs_p(i*8-4+j,:) = obs_p(i*8-8+j,:) + Vz*obs_height(i);
                end

            end

        end
        
        function [lastbas_R,laststp_R,lastjoi,joint_ang ] = colli_avoidance( self, M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option)
        %  求解壁面单步无碰运动
        %[ joint_ang,p_s ] = joint_angle(stp,enp,bap);
        dis_warn = 0.04;  %注意距离为4cm
        dis_safe = 0.02;  %最小安全距离为2cm
        dis = 10000;
        TO_SURFACE_DISTANCE = 0.05;%单步规划时，吸盘抬起的距离


        CurHMatrix  = eye(4);
        enp_o = enp;
        stp_o = stp;
        bap_o = bap;
        bap1 = bap;
         self.wrobot.CurJAs = fir_joi;

        vec_lin = zeros(1,3);
        vec_eff = zeros(1,3);
        num_ob = floor(size(obs_p,1)/8);
        self.wrobot.CurGraspMtx(1:3,4) = bap*1000;
        self.wrobot.CurGraspMtx(1:3,1:3) = bas_R;
        
        if self.draw_flag
        figure (3);
          pp = patch(M_sur(:,1)*1000,M_sur(:,2)*1000,M_sur(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.5,'LineWidth',1);%壁面绘图
           hold on 
           if num_ob>=1          %判断障碍物是否存在
              obp_tran(obs_p);      %障碍物绘图
             for i = 1:num_ob
                      obs_p2(i*8-7:i*8,:) = obs_p(i*8-7:i*8,:);
             end
           end 
        end
        if self.draw_flag
        plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);
        plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);
        axis equal
        end
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
         if num_ob>=1                            %判断障碍物是否存在
         for i = 1:size(obs_p2,1)
             obs_p2(i,:) = (wor_to_bas_R*obs_p(i,:)')'+ wtoba_p;
         end
         %obs_p2 = obs_p;
         end
        %      self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
        %      self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向  
        self.wrobot.TarGraspMtx(1:3,1:3) = stp_R;
        self.wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
        [Flag,sta_ang] = IKine5DNew(self.wrobot);                %求机器人起始构型的关节角
        un_sta = sta_ang(1,5);                               %机器人不变的旋转关节值
        self.wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人目标构型的关节角
        self.wrobot.CurJAs = sta_ang;
        [Flag, end_ang] = IKine5DNew(self.wrobot);      
        if so_option == 2
            diff_eng =360-abs(end_ang(1) - sta_ang(1));
            if end_ang(1) - sta_ang(1)>0
                end_ang(1) = sta_ang(1) - diff_eng;
            else
                 end_ang(1) = sta_ang(1) + diff_eng;
            end     
               Flag
        end  
        self.wrobot.TarJAs =sta_ang;
         % DrawRobotmo(self.wrobot,1);
        self.wrobot.TarJAs =end_ang;
        %  DrawRobotmo(self.wrobot,1);
          end_ang
        %%求解起始点目标点在同一壁面的情形
        %先让机器人上升5cm,对起始点和目标点各提升一段距离用作平缓靠近壁面 
        %2020.2.27改：将这个上升距离参数改成全局，方便修改
         for i = 1:TO_SURFACE_DISTANCE * 100             
            self.wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
            if i > 1 
                self.wrobot.CurJAs = joint_ang(i-1,:);
            end
            [Flag, joint_ang(i,:)] = IKine5DNew(self.wrobot);%Flag是逆运动学是否有解的标志，joint_ang(i,:)是各个关节的角度
         end
         stp = stp + Vn * TO_SURFACE_DISTANCE;   
         bap =  Vn * TO_SURFACE_DISTANCE;

        num_ang = 5;
        i = 0;
        while norm(joint_ang(num_ang,:)-end_ang)>1e-4%如果上升5cm的最后一步跟目标姿态足够近似的话就不进入这个循环

              [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
              joi_pp = (bas_R*joi_pp')'+ bap1*1000;
              if self.draw_flag
              plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
              end
              joi_diffe = abs(end_ang - joint_ang(num_ang,:));   %每个关节角之间的差值足够小，直接到达目标点
            if joi_diffe(1:4)< 3%除了夹持器，每个角度差小于3°的话
                num_ang = num_ang +1;
                joint_ang(num_ang,:) = end_ang;
                break;
            end

             if num_ang>300%如果超过规定步数
                if so_option == 1%如果是壁面规划
                   %   break;
                      %这里不知道什么情况
                      [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
                      return;
                else
                     h=msgbox('规划次数内无结果','warn'); 
                     uiwait(h,2);
                      break;
                end
             end
            %%%%%%%%%%%%%%%%%%如果障碍物不存在
            if num_ob <1    
              [max_diffe,num_j]=max(abs(end_ang-joint_ang(num_ang,:)));
              num_inter = floor(max_diffe/3);
              joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
              num_ang = num_ang + 1; 
              continue;
            end
              dis1 = ones(1,num_ob)*10000;
              dis2 = ones(1,num_ob)*10000;

        %    [ dis,po1,po2] = dis_compute(joi_p/1000,obs_p2,1);      %求机器人末端到障碍物的最小距离
        %     if norm(joi_p)<0.38  %超出机器人工作空间范围
        %             h=msgbox('关节角求解失败','warn'); 
        %           uiwait(h,2);
        %         break;
        %     end

            for j = 1: 4   %求四个关节点
                [ T,joi_p(j,:)] =  Kine5D(joint_ang(num_ang,:) ,j);
            end 
            joi_p = joi_p/1000;
            %靠近壁面避碰策略失效时可用，该距离可斟酌修改（用末端的xy减去目标xy）
            %这里不大懂
            if  norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0])<0.08           
               [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
               num_inter = floor(max_diffe/3);
               joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
               num_ang = num_ang + 1; 
                continue;
            end

           %if norm(joi_p(4,:))>0.586 || norm(joi_p(4,:))<0.38  %超出机器人工作空间范围
            if norm(joi_p(4,:))<0.38  %末端距离基座过近
                if so_option == 1
                  [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
                  return;
            else
                  h=msgbox('末端过近，关节角求解失败','warn'); 
                  uiwait(h,2);
                  break;
                end
            end
        %%%%%%%%以下分别计算吸盘与障碍的距离以及本体对障碍的距离，并在两者中取最小（dis）
        %对所有障碍：
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
                if insidepoly3(obs_p2(8*k-7,:),m_sur) ==0
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

            %距离大于需要注意的距离
            if dis >=dis_warn              
               [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
               num_inter = floor(max_diffe/3);
               joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                num_ang = num_ang + 1;
                continue;
            end
            if dis <= 0.003                %已发生碰撞 
                if so_option == 1
                      [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
                      return;
                else
                      h=msgbox('发生碰撞，规划失败','warn'); 
                      uiwait(h,2);
                      break;
                end
            end

           %2020.2.27跑main的时候没有进入过这个if
           %问一问师兄这是什么意思
           if dis <dis_warn && dis>dis_safe && min(dis1)<min(dis2) %距离大于安全距离但在需要注意的距离内
                joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
                if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 || abs(end_ang(1)-joint_ang(num_ang+1,1))<=2
                    [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                     num_inter = floor(max_diffe/1.5);   %该参数可斟酌修改
                     joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                elseif  joi_p(4,3)-obs_p2(min_ob1*8-3,3)<0.02  
                     joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 3 0 -3 0];
                end
                      num_ang = num_ang +1;
                      continue;
           end

           %距离小于安全距离,判断障碍物上升或者下降是否带来与障碍物距离的缩小，再以此调整主运动方向
           if dis <= dis_safe  && min(dis1)<min(dis2)   %如果末端离障碍物更近
                joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 3 0 -2 0];
                [ dis1kt,po1,po2] = dis_compute(joi_p(4,:),obs_p2(min_ob1*8-7:min_ob1*8,:),1);%用机器人末端点去对最近障碍作测试
                pok = (po1-po2)/norm(po1-po2);
                joi_ps = point_pro_sur(joi_p(4,:),M_sur);

                 if dot(cross(joi_ps-bap,po1 - po2),cross(joi_ps-bap,enp-Vn*TO_SURFACE_DISTANCE-bap))>0         %判断障碍物对末端点的影响和主运动的方向是否相同
                     joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*(1+(dis_safe-dis)*60),0,2,0,0];
                 else
                     joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-(1+(dis_safe-dis)*60),0,2,0,0];
                 end
                 if (end_ang(1)-joint_ang(num_ang-2,1))*(end_ang(1)-joint_ang(num_ang-3,1))<0   %避免主运动方向偏离过大
                     if  norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0])>=0.1    
                         vec_re =( [enp(1),enp(2),0]-[joi_p(4,1),joi_p(4,2),0])/norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0]);
                         next_p = joi_p(4,:) + vec_re * 0.01;
                         self.wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                         self.wrobot.TarGraspMtx(1:3,1:3) = T(1:3,1:3);
                         self.wrobot.CurJAs=joint_ang(num_ang,:);
                         [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(self.wrobot);
                     if Flag == 0
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
                joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*2,0,0,0,0];
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
                     joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-5,0,0,0,0];
                      num_ang = num_ang + 1;
                       continue;
                 end
                 if dis1kt <dis  || num_ang>10 
                     joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-2,0,0,0,0];
                     num_ang = num_ang + 1;
                     continue;
                 end      
                     num_ang = num_ang + 1;
                     continue;
            end

                %%空间障碍物部分
           if dis <dis_warn && dis>dis_safe && min(dis2)<min(dis1) %距离大于安全距离但在需要注意的距离内
                joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
                 if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 || abs(end_ang(1)-joint_ang(num_ang+1,1))<=2
                    [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                     num_inter = floor(max_diffe/3);
                     joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                end
               num_ang = num_ang +1; 
               continue;
           end
             if dis <= dis_safe  && min(dis2)<min(dis1)   %距离小于安全距离
                  [vec_repul] = repul_veccompute(obs_p2(min_ob2*8-7:min_ob2*8,:),m_sur,joi_p);
              effector_p = point_pro_sur(joi_p(4,:),m_sur);
              vec_eff = effector_p/norm(effector_p);
                if dot(vec_repul,Vn)<-0.7
                    if joint_ang(num_ang,2)<-30 || joint_ang(num_ang,4) < -30
                         joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 -0.5 -1 3 0];
                    else
                         joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 -1 2 -1 0];
                    end

                elseif dot(vec_repul,vec_eff)<-0.7
                    joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 1 -2 1 0];    
                end
                 if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 || abs(end_ang(1)-joint_ang(num_ang+1,1))<=3
                    [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                     num_inter = floor(max_diffe/3);
                     joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                end
               num_ang = num_ang + 1;
               continue;
             end
        end

         for i = 1:TO_SURFACE_DISTANCE * 100              %使机器人末端点往Z轴方向靠近TO_SURFACE_DISTANCE * 100cm
              [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
              end_p = end_p - Vn*(0.01)*1000;
              self.wrobot.TarGraspMtx(1:3,1:3) = T(1:3,1:3);
        %        T(1:3,2) = T(1:3,2)*-1;
        %        T(1:3,3) = T(1:3,3)*-1; 
              self.wrobot.TarGraspMtx(1:3,4) = end_p';
              self.wrobot.CurJAs=joint_ang(num_ang+i-1,:);
               [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(self.wrobot);
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

          self.wrobot.TarJAs =joint_ang(6,:);
          
          if self.draw_flag
            DrawRobotmo(self.wrobot,1);
          end
          
           self.wrobot.TarJAs =joint_ang(num_ang-6,:);
           
           if self.draw_flag
            DrawRobotmo(self.wrobot,1);
           end
           
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
           lastjoi=[0,joint_ang(num_ang,4),joint_ang(num_ang,3),joint_ang(num_ang,2),0];
           if self.draw_flag
          plot3(joi_p(:,1), joi_p(:,2),joi_p(:,3),'-k','LineWidth',2);
           end
             % axis([-1000,1000,-1000,1000,0,500])
        end

        function [ joint_ang] = colli_avoidance22( self, M_sur,stp,enp,bap,obs_p,so_option)
        %UNTITLED2 此处显示有关此函数的摘要
        %  生成单步无碰运动，针对机器人在不同壁面间过渡的情况,需要输入基座所在壁面，目标落足点所在壁面，起始落足点所在壁面
            %[ joint_ang,p_s ] = joint_angle(stp,enp,bap);
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
             self.wrobot.TarGraspMtx(1:3,1:3) = sta_to_bas_R;
             Vn = (wor_to_bas_R*Vnm(3,:)')';
             self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
             self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向  
             self.wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
             [ Flag,sta_ang] = IKine5DNew(self.wrobot);                %求机器人起始构型的关节角
            %end

            self.wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人目标构型的关节角
            self.wrobot.CurJAs = sta_ang;
            self.wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
            self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
            self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
            
           % [Flag, end_ang] = IKine5DNew(self.wrobot);   
            [Flag, end_ang] = IKine5D22(self.wrobot);  
             self.wrobot.TarJAs =end_ang;
             if self.draw_flag
               figure (3);
                DrawRobotmo(self.wrobot,1);
                for i = 1:num_msur    %壁面绘图
                    pp = patch(m_sur{i}(:,1)*1000,m_sur{i}(:,2)*1000,m_sur{i}(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.3,'LineWidth',1);
                end
                hold on 
                if num_ob>=1          %判断障碍物是否存在

                    obp_tran(obs_p);      %障碍物绘图

                end
             end
             for i = 1:5             %先让机器人上升5cm,并对起始点和目标点各提升一段距离用作平缓靠近壁面 
                self.wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
                self.wrobot.TarGraspMtx(1:3,1:3) = sta_to_bas_R;
            self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
            self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
                if i > 1 
                    self.wrobot.CurJAs = joint_ang(i-1,:);
                end
                [Flag, joint_ang(i,:)] = IKine5DNew(self.wrobot);
                if Flag == 0
                   h=msgbox('起始位姿有误','warn'); 
                uiwait(h,2);
                break;
                end
             end 
              stp = stp + Vn*0.05;

              if self.draw_flag
              plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);
              plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);
              axis equal
              end
           num_ang = 5;   
        while norm(joint_ang(num_ang,:)-end_ang)>1e-2
             [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
             if self.draw_flag
             plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
             end
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

            joi_p = joi_p/1000;s
            joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
                eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,10);
                eff_p2 = cir_seem_poly( joi_lap,joi_p(4,:)-joi_p(3,:),0.2,10);
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
                 self.wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                self.wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
                self.wrobot.CurJAs=joint_ang(num_ang,:);
                 [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(self.wrobot);
                 if Flag == 0
                     num_up = num_up +1;
                     if num_up == 10
                         h=msgbox('避碰规划失败','warn'); 
                         uiwait(h,2);
                         break;
                     else
                        joint_ang(num_ang+1,:) =  joint_ang(num_ang,:) +[0 2 0 0 0];
                     end
                 end
               num_ang = num_ang + 1;
              end

            if num_ang>200
                h=msgbox('规划次数内无结果','warn'); 
                uiwait(h,2);
            end
        end
        % 
         for i = 1:5               %使机器人末端点往Z轴方向靠近5cm
              [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
              end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.01*1000;

              self.wrobot.TarGraspMtx(1:3,4) = end_p';
              self.wrobot.CurJAs=joint_ang(num_ang+i-1,:);
               self.wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
            self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
            self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
               [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(self.wrobot);
         end
              num_ang = num_ang + 5;
              joint_ang = [sta_ang;joint_ang];
         num_joi = size(joint_ang,1);
          self.wrobot.TarJAs =joint_ang(6,:);
          if self.draw_flag
          DrawRobotmo(self.wrobot,1);
          end
           self.wrobot.TarJAs =joint_ang(num_joi,:);
           if self.draw_flag
           DrawRobotmo(self.wrobot,1);
           end
           end_ang
          for i = 1: num_joi
              [ T,joi_po(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
          end
          if self.draw_flag
              plot3(joi_po(:,1), joi_po(:,2),joi_po(:,3),'-k','LineWidth',2);
              axis([-2000,2000,-2000,2000,0,2000])
          end
         end
        
        function [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance33( self, M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option)
            %  生成单步无碰运动，针对机器人在不同壁面间过渡的情况,需要输入基座所在壁面，目标落足点所在壁面，起始落足点所在壁面
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

            fir_joi=[0,-23.0048561393523,-27.1477662270062,129.071883883613,0];
             self.wrobot.CurJAs = fir_joi;
             num_msur=size(M_sur,2);   %求机器人运动过程中需要经过的壁面及基座标系，并对环境基于基座标系作转化，设第一个壁面为基座标所在壁面，第二个壁面为目标落足点所在壁面，第三个壁面为起始落足点所在壁面
             m_sur = cell(num_msur,1);        %多边形点进行坐标系转换后的储存
             num_ob = floor(size(obs_p,1)/8);
            obs_p2 = obs_p;
               self.wrobot.CurGraspMtx(1:3,4) = bap*1000;
               self.wrobot.CurGraspMtx(1:3,1:3) = bas_R;
               
               if self.draw_flag
            figure (3);
               for i = 1:num_msur    %壁面绘图
               pp = patch(M_sur{i}(:,1)*1000,M_sur{i}(:,2)*1000,M_sur{i}(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.3,'LineWidth',1);
               end
                hold on 
              if num_ob>=1          %判断障碍物是否存在
                 obp_tran(obs_p);      %障碍物绘图
              end
              if self.draw_flag
                  plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);
                  plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);
                  axis equal
              end
               end
               
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
               % bas_R = [bas_x',bas_y',bas_z'];
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
                 enp = enp + (wor_to_bas_R*Vnm(2,:)')'*0.05;       %目标吸附点的辅助点

                sta_x = (M_sur{3}(2,:)-M_sur{3}(1,:))/norm(M_sur{3}(2,:)-M_sur{3}(1,:));   %求机器人运动的起点坐标系
                sta_z = Vnm(3,:);
                sta_y = cross(sta_z,sta_x);
               % sta_R = [sta_x',sta_y',sta_z'];
                sta_R = stp_R;                   %起点坐标系在基坐标系下的表示

               % sta_to_bas_R = bas_R'*sta_R;    %起点坐标系在基坐标系下的表示

                % self.wrobot.TarGraspMtx(1:3,1:3) = sta_to_bas_R;
                self.wrobot.TarGraspMtx(1:3,1:3) = stp_R;
                 Vn = (wor_to_bas_R*Vnm(3,:)')';
            %     self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
            %     self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向  
                 self.wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
                 [ Flag,sta_ang] = IKine5DNew(self.wrobot);                %求机器人起始构型的关节角
                %end
                 un_sta = sta_ang(1,5); 
                self.wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人目标构型的关节角
                self.wrobot.CurJAs = sta_ang;
                self.wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
                 self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
                 self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
                 [Flag, end_ang] = IKine5DNew(self.wrobot);   
                 if Flag == 0 
                      h=msgbox('辅助目标点生成不成功','warn'); 
                    uiwait(h,2);
                    return;
                  end
             %[Flag, end_ang] = IKine5D22(self.wrobot);   
                if so_option == 2
            %         diff_eng =360-abs(end_ang(1) - sta_ang(1));
            %         if end_ang(1) - sta_ang(1)>0
            %             end_ang(1) = sta_ang(1) - diff_eng;
            %         else
            %              end_ang(1) = sta_ang(1) + diff_eng;
            %         end
                       [Flag, end_ang] = IKine5D22(self.wrobot);
                end


                 self.wrobot.TarJAs =sta_ang;
                %  DrawRobotmo(self.wrobot,1);
                self.wrobot.TarJAs =end_ang;
                %  DrawRobotmo(self.wrobot,1);
                end_ang
                 for i = 1:5             %先让机器人上升5cm,并对起始点和目标点各提升一段距离用作平缓靠近壁面 
            %         self.wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
            %         self.wrobot.TarGraspMtx(1:3,1:3) = sta_R;

                    if i > 1 
                        self.wrobot.CurJAs = joint_ang(i-1,:);

                    %[Flag, joint_ang(i,:)] = IKine5DNew(self.wrobot);
                  %  if Flag == 0
            %            h=msgbox('起始上升位姿有误','warn'); 
            %         uiwait(h,2);
            %         break;
                     joint_ang(i,:) = joint_ang(i-1,:) + [0 -4 0 0 0];

                    %end
                    else
                      joint_ang(i,:)  = sta_ang;   
                    end 
                 end
                  stp = stp + Vn*0.05;


            % 
              % obs2 = [-0.6 -0.5 0.3;-0.5 -0.5 0.3;-0.5 -0.2 0.3;-0.6 -0.2 0.3;-0.6 -0.5 0.5;-0.5 -0.5 0.5;-0.5 -0.2 0.5;-0.6 -0.2 0.5];
              % obs2 = [-0.6 -0.5 0;-0.5 -0.5 0;-0.5 -0.2 0;-0.6 -0.2 0;-0.6 -0.5 0.2;-0.5 -0.5 0.2;-0.5 -0.2 0.2;-0.6 -0.2 0.2];
              % obs_p = [-0.65 -0.5 0;-0.55 -0.5 0;-0.55 -0.2 0;-0.65 -0.2 0;-0.65 -0.5 0.2;-0.55 -0.5 0.2;-0.55 -0.2 0.2;-0.65 -0.2 0.2];

            %obs2 = [0.2 0.2 0.37;0.25 0.2 0.37;0.25 0.3 0.37;0.2 0.3 0.37;0.2 0.2 0.5;0.25 0.2 0.5;0.25 0.3 0.5;0.2 0.3 0.5];
            %   %[obs2] = enlarge_border(obs_border) ;
            % %   obp_tran(obs_border);
              %obp_tran(obs2);
            %  self.wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人到达目标点的关节角
            %  [Flag, end_ang] = IKine5DNew(self.wrobot);

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
            end_ang(1,1) = end_ang(1,1)+120;
               num_ang = 5;   
            while norm(joint_ang(num_ang,:)-end_ang)>1e-2
                 [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
            %      plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
            %         [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
                  joi_pp = (bas_R*joi_pp')'+ bap_o*1000;
                  if self.draw_flag
                  plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
                  end
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

                   if norm(joi_p(4,:))<0.4  %超出机器人工作空间范围
            %        [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
            %        num_inter = floor(max_diffe/2);
                    joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
                   joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:)+  [0,-2,4,-2,0];
                    num_ang = num_ang + 1; 
                    continue;
            %              if so_option == 1
            %          
            %                  [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
            %                   return;
            %               else
            %             h=msgbox('距离基座过近，关节角求解失败','warn'); 
            %           uiwait(h,2);
            %             break;
            %              end
                   end
                    if  norm(joi_p(4,:)-enp)<0.3           %靠近壁面避碰策略失效时可用
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
                      if so_option == 1
                       %   break;
                          [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance33( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                          return;
                      else
                       h=msgbox('发生碰撞，规划失败','warn'); 
                    uiwait(h,2);
                    break;
                      end
                  elseif state(num_ang-4) == 1
                       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                   num_inter = floor(max_diffe/3);
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                    num_ang = num_ang + 1; 
                  elseif state(num_ang-4) == 2 
                       %vec_repul = vec_eff/norm(vec_eff);    
                     next_p = joi_p(4,:) + vec_re * 0.01;
                     self.wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                    self.wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
                    self.wrobot.CurJAs=joint_ang(num_ang,:);
                     [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(self.wrobot);
                     if Flag == 0
                         num_up = num_up +1;
                         if num_up == 10
                          if so_option == 1
                             % break;
                              so = 2;
                              [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance33( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                              return;
                          else
                                 h=msgbox('避碰规划失败','warn'); 
                                 uiwait(h,2);
                        break;
                          end
                         else
                            joint_ang(num_ang+1,:) =  joint_ang(num_ang,:) +[0 2 0 0 0];
                         end
                     end
                   num_ang = num_ang + 1;

                  end

                if num_ang>200   
                     if so_option == 1
                       %  break;
                              so = 2;
                              [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance33( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                              return;
                          else
                                 h=msgbox('规划次数内无结果','warn'); 
                                 uiwait(h,2);
                        break;
                      end
                end

            end
            num_ang = size(joint_ang,1);
            end_ang(1,1) = end_ang(1,1)-120;
            while norm(joint_ang(num_ang,:)-end_ang)>1e-2
                 [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
            %      plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
            %         [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
                  joi_pp = (bas_R*joi_pp')'+ bap_o*1000;
                  if self.draw_flag
                  plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
                  end
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

                   if norm(joi_p(4,:))<0.4  %超出机器人工作空间范围
            %        [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
            %        num_inter = floor(max_diffe/2);
                    joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
                   joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:)+  [0,-2,4,-2,0];
                    num_ang = num_ang + 1; 
                    continue;
            %              if so_option == 1
            %          
            %                  [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
            %                   return;
            %               else
            %             h=msgbox('距离基座过近，关节角求解失败','warn'); 
            %           uiwait(h,2);
            %             break;
            %              end
                   end
                    if  norm(joi_p(4,:)-enp)<0.3           %靠近壁面避碰策略失效时可用
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
                      if so_option == 1
                       %   break;
                          [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance33( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                          return;
                      else
                       h=msgbox('发生碰撞，规划失败','warn'); 
                    uiwait(h,2);
                    break;
                      end
                  elseif state(num_ang-4) == 1
                       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                   num_inter = floor(max_diffe/3);
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                    num_ang = num_ang + 1; 
                  elseif state(num_ang-4) == 2 
                       %vec_repul = vec_eff/norm(vec_eff);    
                     next_p = joi_p(4,:) + vec_re * 0.01;
                     self.wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                    self.wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
                    self.wrobot.CurJAs=joint_ang(num_ang,:);
                     [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(self.wrobot);
                     if Flag == 0
                         num_up = num_up +1;
                         if num_up == 10
                          if so_option == 1
                             % break;
                              so = 2;
                              [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance33( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                              return;
                          else
                                 h=msgbox('避碰规划失败','warn'); 
                                 uiwait(h,2);
                        break;
                          end
                         else
                            joint_ang(num_ang+1,:) =  joint_ang(num_ang,:) +[0 2 0 0 0];
                         end
                     end
                   num_ang = num_ang + 1;

                  end

                if num_ang>200   
                     if so_option == 1
                       %  break;
                              so = 2;
                              [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance33( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                              return;
                          else
                                 h=msgbox('规划次数内无结果','warn'); 
                                 uiwait(h,2);
                        break;
                      end
                end

            end
            % 
             for i = 1:5               %使机器人末端点往Z轴方向靠近5cm
                  [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
                  end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.01*1000;
                  self.wrobot.TarGraspMtx(1:3,4) = end_p';
                  self.wrobot.CurJAs=joint_ang(num_ang+i-1,:);
                   self.wrobot.TarGraspMtx(1:3,1:3) =  T(1:3,1:3);
               % self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
               % self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
                   [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(self.wrobot);
             end
                  num_ang = num_ang + 5;
                  joint_ang = [sta_ang;joint_ang];
                  num_joi = size(joint_ang,1);
                  joint_ang(:,5) = un_sta;
                   self.wrobot.TarJAs =joint_ang(1,:);
                   if self.draw_flag
                    DrawRobotmo(self.wrobot,1);
                   end
                   self.wrobot.TarJAs =joint_ang(num_ang,:);
                   if self.draw_flag
                   DrawRobotmo(self.wrobot,1);
                   end
               %end_ang
              for i = 1: num_joi
                  [ T,joi_po(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
                   joi_po(i,:) = (bas_R*joi_po(i,:)')'+bap_o*1000;
              end
                T(1:3,2) = T(1:3,2)*-1;
               T(1:3,3) = T(1:3,3)*-1;
              lastbas_R = bas_R*T(1:3,1:3);
            %    lastbas_R(1:3,2) = lastbas_R(1:3,2)*-1;
            %    lastbas_R(1:3,3) = lastbas_R(1:3,3)*-1;
            %     T(1:3,2) = T(1:3,2)*-1;
            %    T(1:3,3) = T(1:3,3)*-1;
               laststp_R = T(1:3,1:3)';
            %    laststp_R = bas_R*lastbas_R';
               laststp_R(1:3,2) =  laststp_R(1:3,2)*-1;
               laststp_R(1:3,3) =  laststp_R(1:3,3)*-1;
               lastjoi=[0,joint_ang(num_ang,4),joint_ang(num_ang,3),joint_ang(num_ang,2),0];
               if self.draw_flag   
               plot3(joi_po(:,1), joi_po(:,2),joi_po(:,3),'-g','LineWidth',2);
               end
               %   axis([-2000,2000,-2000,2000,0,2000])
        end

        function [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( self, M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option)
            %  求解壁面过渡单步无碰运动，针对机器人在不同壁面间过渡的情况,需要输入基座所在壁面，目标落足点所在壁面，起始落足点所在壁面等参数
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
             self.wrobot.CurJAs = fir_joi;
             num_msur=size(M_sur,2);   %求机器人运动过程中需要经过的壁面及基座标系，并对环境基于基座标系作转化，设第一个壁面为基座标所在壁面，第二个壁面为目标落足点所在壁面，第三个壁面为起始落足点所在壁面
             m_sur = cell(num_msur,1);        %多边形点进行坐标系转换后的储存
             num_ob = floor(size(obs_p,1)/8);
            obs_p2 = obs_p;
               self.wrobot.CurGraspMtx(1:3,4) = bap*1000;
               self.wrobot.CurGraspMtx(1:3,1:3) = bas_R;
               if self.draw_flag
                    figure (3);
                    for i = 1:num_msur    %壁面绘图
                        pp = patch(M_sur{i}(:,1)*1000,M_sur{i}(:,2)*1000,M_sur{i}(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.3,'LineWidth',1);
                    end
                    hold on 
                    if num_ob>=1          %判断障碍物是否存在
                        obp_tran(obs_p);      %障碍物绘图
                    end
                        plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);
                        plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);
                        axis equal
                end

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
               % bas_R = [bas_x',bas_y',bas_z'];
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
                 enp = enp + (wor_to_bas_R*Vnm(2,:)')'*0.05;       %目标吸附点的辅助点

                sta_x = (M_sur{3}(2,:)-M_sur{3}(1,:))/norm(M_sur{3}(2,:)-M_sur{3}(1,:));   %求机器人运动的起点坐标系
                sta_z = Vnm(3,:);
                sta_y = cross(sta_z,sta_x);
               % sta_R = [sta_x',sta_y',sta_z'];
                sta_R = stp_R;                   %起点坐标系在基坐标系下的表示

               % sta_to_bas_R = bas_R'*sta_R;    %起点坐标系在基坐标系下的表示

                % self.wrobot.TarGraspMtx(1:3,1:3) = sta_to_bas_R;
                self.wrobot.TarGraspMtx(1:3,1:3) = stp_R;
                 Vn = (wor_to_bas_R*Vnm(3,:)')';
            %     self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
            %     self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向  
                 self.wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
                 [ Flag,sta_ang] = IKine5DNew(self.wrobot);                %求机器人起始构型的关节角
                %end
                 un_sta = sta_ang(1,5); 
                self.wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %求机器人目标构型的关节角
                self.wrobot.CurJAs = sta_ang;
                self.wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
                 self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
                 self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
                 [Flag, end_ang] = IKine5DNew(self.wrobot);   
                 if Flag == 0 
                      h=msgbox('辅助目标点生成不成功','warn'); 
                    uiwait(h,2);
                    return;
                  end
             %[Flag, end_ang] = IKine5D22(self.wrobot);   
                if so_option == 2
                       [Flag, end_ang] = IKine5D22(self.wrobot);
                       Flag
                end


                 self.wrobot.TarJAs =sta_ang;
               %   DrawRobotmo(self.wrobot,1);
                self.wrobot.TarJAs =end_ang;
               %   DrawRobotmo(self.wrobot,1);
               % end_ang
                 for i = 1:5             %先让机器人上升5cm,并对起始点和目标点各提升一段距离用作平缓靠近壁面 
                    self.wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
                    self.wrobot.TarGraspMtx(1:3,1:3) = sta_R;
            %      self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
            %      self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
                    if i > 1 
                        self.wrobot.CurJAs = joint_ang(i-1,:);
                    end
                    [Flag, joint_ang(i,:)] = IKine5DNew(self.wrobot);
                    if Flag == 0
            %            h=msgbox('起始上升位姿有误','warn'); 
            %         uiwait(h,2);
            %         break;
                     joint_ang(i,:) = joint_ang(i-1,:) + [0 3 0 0 0];
                    end
                 end 
                  stp = stp + Vn*0.05;

               num_ang = 5;   
            while norm(joint_ang(num_ang,:)-end_ang)>1e-4
                 [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
            %      plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
            %         [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
                  joi_pp = (bas_R*joi_pp')'+ bap_o*1000;
                  if self.draw_flag
                  plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
                  end
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
                eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);
                eff_p2 = cir_seem_poly( joi_lap,joi_p(4,:)-joi_p(3,:),0.2,10);

                   if norm(joi_p(4,:))<0.4  %末端距离基座过近
                    joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
                   joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:)+  [0,-2,4,-2,0];
                    num_ang = num_ang + 1; 
                    continue;
                   end
                    if  norm(joi_p(4,:)-enp)<0.2           %靠近壁面避碰策略失效时可用
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
                      if so_option == 1
                       %   break;
                          [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                          return;
                      else
                       h=msgbox('发生碰撞，规划失败','warn'); 
                    uiwait(h,2);
                    break;
                      end
                  elseif state(num_ang-4) == 1
                       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                   num_inter = floor(max_diffe/3);
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
                    num_ang = num_ang + 1; 
                  elseif state(num_ang-4) == 2 
                       %vec_repul = vec_eff/norm(vec_eff);    
                     next_p = joi_p(4,:) + vec_re * 0.01;
                     self.wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                    self.wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
                    self.wrobot.CurJAs=joint_ang(num_ang,:);
                     [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(self.wrobot);
                     if Flag == 0
                         num_up = num_up +1;
                         if num_up == 10
                          if so_option == 1
                             % break;

                              [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                              return;
                          else
                                 h=msgbox('避碰规划失败','warn'); 
                                 uiwait(h,2);
                        break;
                          end
                         else
                            joint_ang(num_ang+1,:) =  joint_ang(num_ang,:) +[0 2 0 0 0];
                         end
                     end
                   num_ang = num_ang + 1;

                  end
                if num_ang>200   
                     if so_option == 1
                       %  break;

                              [lastbas_R,laststp_R,lastjoi,joint_ang] = self.colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                              return;
                          else
                                 h=msgbox('规划次数内无结果','warn'); 
                                 uiwait(h,2);
                        break;
                      end
                end
            end
            % 
             for i = 1:5               %使机器人末端点往Z轴方向靠近5cm
                  [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
                  end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.01*1000;
                  self.wrobot.TarGraspMtx(1:3,4) = end_p';
                  self.wrobot.CurJAs=joint_ang(num_ang+i-1,:);
                   self.wrobot.TarGraspMtx(1:3,1:3) =  T(1:3,1:3);
               % self.wrobot.TarGraspMtx(1:3,2)=self.wrobot.TarGraspMtx(1:3,2)*-1;
               % self.wrobot.TarGraspMtx(1:3,3)=self.wrobot.TarGraspMtx(1:3,3)*-1;    %调整末端坐标系方向
                   [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(self.wrobot);
                   if Flag ==0
                        h=msgbox('目标点下降无解','warn'); 
                                 uiwait(h,2);
                        return;
                   end
             end
                  num_ang = num_ang + 6;
                  joint_ang = [sta_ang;joint_ang];
                  num_joi = size(joint_ang,1);
                  joint_ang(:,5) = un_sta;
                   self.wrobot.TarJAs =joint_ang(1,:);
                   if self.draw_flag
                   DrawRobotmo(self.wrobot,1);
                   end
                   self.wrobot.TarJAs =joint_ang(num_ang,:);
                   if self.draw_flag
                   DrawRobotmo(self.wrobot,1);
                   end
             %end_ang
             num_pro = num_joi - 10;                  %%对路径做平滑处理
             num_pro = floor(num_pro/2);
             for i = 1: num_pro
                 dis1 = 10000;
                dis2 = 10000;
                vec_lin = zeros(1,3);
                vec_eff = zeros(1,3);
                joint_ang2 = (joint_ang(i*2-1+5,:)+joint_ang(i*2+1+5,:))/2;

                if norm(joint_ang(i*2+5,:)-joint_ang2)<0.01
                    continue;
                end
                for j = 1: 4   %求四个关节点
                     [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
                end 

                joi_p = joi_p/1000;
                joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
                eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,10);
                  for k= 1: num_msur
                     for j=1:3            %求机器人本体连杆距离壁面的最小距离
                         if j == 3
                             [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],m_sur{k},4);
                         else
                         [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],m_sur{k},4);
                         end
                     if dist < dis1
                     dis1 = dist;
                     vec_lin = po1 - po2;
                     end
                     end 
                  end
                  if num_ob >0
                      for k = 1:num_ob
                  for j = 1:3       %求机器人本体连杆到障碍物的最小距离
                      if j == 3
                          [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],obs_p(k*8-7:k*8,:),2);
                      else
                       [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p(k*8-7:k*8,:),2);
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
                 for k = 1:num_ob 

                        [ dist,po1,po2] = dis_compute(joi_p(4,:),obs_p(k*8-7:k*8,:),1);
                        dist = dist - 0.2;
                     if dist < dis2
                         dis2 = dist;
                          vec_eff = po1 - po2;
                     end
                 end
            end       
                  for k= 1: num_msur
                      [ dist,po1,po2] = dis_compute(eff_p,m_sur{k},5);
                     if dist < dis2 
                     dis2 = dist ;
                     vec_eff = po1 - po2;
                     end
                  end 
                  [state1,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,joi_p(4,:),enp);
                  if state1 > 0
                      joint_ang(2*i+5,:) = joint_ang2;
                  else
                      continue;
                  end
             end
             for i = 1: num_pro-1
                 dis1 = 10000;
                dis2 = 10000;
                vec_lin = zeros(1,3);
                vec_eff = zeros(1,3);
                joint_ang2 = (joint_ang(i*2+5,:)+joint_ang(i*2+2+5,:))/2;

                if norm(joint_ang(i*2+1+5,:)-joint_ang2)<0.01
                    continue;
                end
                for j = 1: 4   %求四个关节点
                     [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
                end 

                joi_p = joi_p/1000;
                joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
                eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,10);
                  for k= 1: num_msur
                     for j=1:3            %求机器人本体连杆距离壁面的最小距离
                         if j == 3
                             [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],m_sur{k},4);
                         else
                         [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],m_sur{k},4);
                         end
                     if dist < dis1
                     dis1 = dist;
                     vec_lin = po1 - po2;
                     end
                     end 
                  end
                  if num_ob >0
                      for k = 1:num_ob
                  for j = 1:3       %求机器人本体连杆到障碍物的最小距离
                      if j == 3
                          [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],obs_p(k*8-7:k*8,:),2);
                      else
                       [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p(k*8-7:k*8,:),2);
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
                 for k = 1:num_ob 

                        [ dist,po1,po2] = dis_compute(joi_p(4,:),obs_p(k*8-7:k*8,:),1);
                        dist = dist - 0.2;
                     if dist < dis2
                         dis2 = dist;
                          vec_eff = po1 - po2;
                     end
                 end
            end       
                  for k= 1: num_msur
                      [ dist,po1,po2] = dis_compute(eff_p,m_sur{k},5);
                     if dist < dis2 
                     dis2 = dist ;
                     vec_eff = po1 - po2;
                     end
                  end 
                  [state1,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,joi_p(4,:),enp);
                  if state1 > 0
                      joint_ang(2*i+1+5,:) = joint_ang2;
                  else
                      continue;
                  end
             end
              for i = 1: num_joi
                  [ T,joi_po(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
                   joi_po(i,:) = (bas_R*joi_po(i,:)')'+bap_o*1000;
              end
              T(1:3,4)
                T(1:3,2) = T(1:3,2)*-1;
               T(1:3,3) = T(1:3,3)*-1;
              lastbas_R = bas_R*T(1:3,1:3);
            %    lastbas_R(1:3,2) = lastbas_R(1:3,2)*-1;
            %    lastbas_R(1:3,3) = lastbas_R(1:3,3)*-1;
            %     T(1:3,2) = T(1:3,2)*-1;
            %    T(1:3,3) = T(1:3,3)*-1;
               laststp_R = T(1:3,1:3)';
            %    laststp_R = bas_R*lastbas_R';
               laststp_R(1:3,2) =  laststp_R(1:3,2)*-1;
               laststp_R(1:3,3) =  laststp_R(1:3,3)*-1;
               lastjoi=[0,joint_ang(num_ang,4),joint_ang(num_ang,3),joint_ang(num_ang,2),0];
               if self.draw_flag
                  plot3(joi_po(:,1), joi_po(:,2),joi_po(:,3),'-g','LineWidth',2);
               end
                  %   axis([-2000,2000,-2000,2000,0,2000])
            end
        
        function [] = motionplan(self)
            expand_surface = self.expand_surface();
            surface_step_index =zeros(1,50);

            %三个点构成一个动作，记录下这三个点所在平面
            for j = 1:size(self.wall_planning_points,1) - 2
                start_point = self.wall_planning_points(j,:);%start point
                base_point = self.wall_planning_points(j+1,:);%base point 
                end_point = self.wall_planning_points(j+2,:);%end point
                for i= 1:self.num_surfaces
                    if insidepoly3(base_point,self.surface{i}) ==1
                        self.surface_step{1} = self.surface{i};
                        surface_step_index(1) = i;
                    end
                    if insidepoly3(end_point,self.surface{i}) ==1
                          self.surface_step{2} = self.surface{i};
                          surface_step_index(2) = i;
                    end
                    if insidepoly3(start_point,self.surface{i}) ==1
                      self.surface_step{3} = self.surface{i};
                      surface_step_index(3) = i;
                    end       
                end

                step_option = self.step_type(j);
                base_surface = self.surface_step{1};

                %计算基座坐标系
                Vnm=cross(base_surface(2,:)-base_surface(1,:),base_surface(3,:)-base_surface(2,:));   %计算多边形法向量
                Vnm=Vnm/norm(Vnm);
                bas_x = (base_surface(2,:)-base_surface(1,:))/norm(base_surface(2,:)-base_surface(1,:));   %求机器人运动的基坐标系
                bas_z = Vnm; 
                bas_y = cross(bas_z,bas_x);
                bas_R_w_b = [bas_x',bas_y',bas_z'];%R_w_b

                if step_option ==1
                    stp_R_w_s = eye(3);
                    stp_R_w_s(1:3,2) = stp_R_w_s(1:3,2)*-1;
                    stp_R_w_s(1:3,3) = stp_R_w_s(1:3,3)*-1;
                else
                    %求机器人运动的起点坐标系
                    %先求出起始壁面的姿态，转换到基座坐标系下，然后将y、z轴方向取反
                    sta_x = (self.surface_step{3}(2,:)-self.surface_step{3}(1,:))/norm(self.surface_step{3}(2,:)-self.surface_step{3}(1,:));   
                    Vnm3=cross(self.surface_step{3}(2,:)-self.surface_step{3}(1,:),self.surface_step{3}(3,:)-self.surface_step{3}(2,:));   %计算多边形法向量
                    Vnm3=Vnm3/norm(Vnm3);
                    sta_z = Vnm3;
                    sta_y = cross(sta_z,sta_x); 
                    stp_R2 =[sta_x',sta_y',sta_z'];
                    stp_R_w_s =  bas_R_w_b'*stp_R2;%R_b_s
                    stp_R_w_s(1:3,2) = stp_R_w_s(1:3,2)*-1;
                    stp_R_w_s(1:3,3) = stp_R_w_s(1:3,3)*-1;
                end

                first_joint = [0  20  -40  20 0];
                % [-23.0048561393523,-27.1477662270062,129.071883883613]
                %这个是上一次的基座、起始壁面姿态
                if j > 1
                    bas_R_w_b = last_bas_R;
                    stp_R_w_s = last_stp_R;
                    first_joint = last_joint;
                end
                self.Ba(j,:)=base_point;
                self.Ba_R{j}=bas_R_w_b;

                if step_option ==1    %%壁面单步运动规划
                    testobj =base_point + Vnm*self.link_1;
                    obs_p= [];

                    %将障碍物坐标点转换到壁面坐标系下（这个数组obs_p存放了全部壁面障碍点）
                    num_obstructs = 1;
                    for i = 1:self.num_surfaces%求解机器人工作空间内的有效障碍物
                        if self.obstruct_height{i} == 0
                            continue;
                        end
                        num_ob2 = length(self.obstruct_height{i});
                        obs_p(num_obstructs*8-7:(num_obstructs + num_ob2 - 1)*8,:)  = map_obs( expand_surface{i},self.obstruct_borders{i},self.obstruct_height{i} );
                        % obs_p(1:8,:) = map_obs( expand_surface{i},self.obstruct_borders{i},self.obstruct_height{i} );
                         num_obstructs = num_obstructs + num_ob2;
                    end
                    num_obstructs = 1;
                    self.valiable_obstruct_index = [] ;
                    %判断机器人工作空间内是否有障碍物
                    if size(obs_p,1)>0
                        for i=1:floor(size(obs_p,1)/8)
                            [ dis,po1,po2] = dis_compute(testobj,obs_p(i*8-7:i*8,:),1);
                            if dis <= 2*self.link_2+self.link_1 %伸直状态
                                self.valiable_obstruct_index(num_obstructs) = i;
                               % obs_p2(num_obstructs*8+1:num_obstructs*8+8,:) = obs_p(i*8-7:i*8,:);
                                num_obstructs = num_obstructs +1;
                            end
                        end
                    end
                    if size(self.valiable_obstruct_index,1) ==0
                        obs_p2 = zeros(4,3);
                    else   
                        obs_p2 = zeros(length(self.valiable_obstruct_index)*8,3);
                        for i = 1:length(self.valiable_obstruct_index)
                            obs_p2(i*8-7:i*8,:) = obs_p(self.valiable_obstruct_index(i)*8-7:self.valiable_obstruct_index(i)*8,:);
                        end
                    end
                %[self.joint_ang ] = colli_avoidance( map_border,start_point,end_point,base_point,obs_p2 );
                %[last_bas_R self.joint_ang{j}] = colli_avoidance(  self.surface_step{1},start_point,end_point,base_point,obs_p2,bas_R_w_b ); 
                    tic
                    [last_bas_R,last_stp_R,last_joint,self.joint_ang{j}] = self.colli_avoidance( base_surface,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
                    toc
                else            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%壁面过渡运动规划
                    obs_p= [];
                    %%壁面过渡运动规划
                    testobj =base_point + Vnm*self.link_1;
                    num_obsur = 3;
                  %  obs_p = 0;

                    %2020.1.11注释
                    if any(surface_step_index(1:3) == 0)
                       % self.joint_ang = 0;
                          h=msgbox('没有求解到落足点对应的壁面','warn'); 
                              uiwait(h,2);
                        return;
                    end

                    for i = 1:self.num_surfaces 
                        [ dis,po1,po2] = dis_compute(testobj,self.surface{i},3);
                        if dis <= 2*self.link_2+self.link_1 
                            if any(surface_step_index(1:3)==i)
                                continue;
                            else
                                surface_step_index(num_obsur +1) = i;
                                self.surface_step{num_obsur+1} = self.surface{i};
                                num_obsur = num_obsur +1;
                            end
                        end
                    end
                    num_obstructs = 1;
                    for i = 1:self.num_surfaces                            %求解机器人工作空间内的有效障碍物
                        if self.obstruct_height{i} == 0
                            continue;
                        end
                        num_ob2 = length(self.obstruct_height{i});
                        obs_p(num_obstructs*8-7:(num_obstructs + num_ob2 - 1)*8,:)  = map_obs( expand_surface{i},self.obstruct_borders{i},self.obstruct_height{i} );
                        % obs_p(1:8,:) = map_obs( expand_surface{i},self.obstruct_borders{i},self.obstruct_height{i} );
                         num_obstructs = num_obstructs + num_ob2;
                    end
                 %obs_p
                    num_obstructs = 1;
                    if size(obs_p,1)>1
                        for i=1:floor(size(obs_p,1)/8)
                            [ dis,po1,po2] = dis_compute(testobj,obs_p(i*8-7:i*8,:),1);
                            if dis <= 2*self.link_2+self.link_1
                                self.valiable_obstruct_index(num_obstructs) = i;
                               % obs_p2(num_obstructs*8+1:num_obstructs*8+8,:) = obs_p(i*8-7:i*8,:);
                                num_obstructs = num_obstructs +1;
                            end
                        end
                    end
                    if self.valiable_obstruct_index ==0
                        obs_p2 = zeros(4,3);   %表示工作空间内无障碍物
                    else  
                        obs_p2 = zeros(length(self.valiable_obstruct_index)*8,3);
                        for i = 1:length(self.valiable_obstruct_index)
                            obs_p2(i*8-7:i*8,:) = obs_p(self.valiable_obstruct_index(i)*8-7:self.valiable_obstruct_index(i)*8,:);
                        end
                    end
                end

                % self.surface_step={expand_surface{1},expand_surface{2},expand_surface{1}};
                %[self.joint_ang{j}] = colli_avoidance22( self.surface_step,start_point,end_point,base_point,obs_p2,1 );
                tic
                if j ==16
                    [last_bas_R,last_stp_R,last_joint,self.joint_ang{j}] = self.colli_avoidance2( self.surface_step,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,2);
                    continue;
                elseif j ==17
                    [last_bas_R,last_stp_R,last_joint,self.joint_ang{j}] = self.colli_avoidance33( self.surface_step,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
                else
                    [last_bas_R,last_stp_R,last_joint,self.joint_ang{j}] = self.colli_avoidance2( self.surface_step,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
                end
                toc
            %[last_bas_R,last_stp_R,last_joint,self.joint_ang{j}] = colli_avoidance2( self.surface_step,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
            %[last_bas_R,last_stp_R,last_joint,self.joint_ang{j}] = colli_avoidance33( self.surface_step,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
            end
        end
    end
end   % classdef

function [ dis,po1,po2] = dis_compute(testobj,obsp,option)
%  机器人与障碍物之间求距与距离最近点,testobj为测距对象，obsp为测距障碍物  ,po1为测试的点，po2为障碍物上的点
%option为选择何种元素测距，包括点与障碍物求距（1），线段与障碍物求距（2），点与壁面求距(3)，线段与壁面求距(4),多边形与多边形求距（5）,多边形与障碍物求距（6）
%poly =[0 0 2;5 0 2;5 4 2];
% poly2=[-0.7 -0.3 0.1;-0.7 -0.4 0.1;-0.5 -0.6,1];
% point = [6 3 3];
% lineseg=[8 2 0;8 2 7];
%obsp = [-0.6 -0.5 0;-0.5 -0.5 0;-0.5 -0.2 0;-0.6 -0.2 0;-0.6 -0.5 0.2;-0.5 -0.5 0.2;-0.5 -0.2 0.2;-0.6 -0.2 0.2];
dis = 10000;
po1 = zeros(1,3);
po2 = zeros(1,3);
if option == 1
    [obs_sur] =obp_tran(obsp);
    for i=1:5
        num_p = size(obs_sur,2)/3;
        for j=1:num_p
        poly(j,:) = obs_sur(i,j*3-2:j*3,1);
        end
        [dist,p1,p2] = pointtopoly(testobj,poly);
        if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
    end
end
% for i=1:5                                 %画出结果
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%          plot3(sur_x, sur_y,sur_z,'-r','LineWidth',2);
%          hold on;
% plot3(testobj(1),testobj(2),testobj(3),'*b','LineWidth',3);
%  plot3(po1(1,1),po1(1,2),po1(1,3),'*b','LineWidth',3);
%  plot3(po2(1,1),po2(1,2),po2(1,3),'*k','LineWidth',3);
%mArrow3([po2(1,1),po2(1,2),po2(1,3)],[po1(1,1),po1(1,2),po1(1,3)],'color','m','stemWidth',25,'facealpha',0.7);
%end
    if option == 2
         [obs_sur] =obp_tran(obsp);
      for i=1:5
        num_p = size(obs_sur,2)/3;
        for j=1:num_p
        poly(j,:) = obs_sur(i,j*3-2:j*3,1);
        end
        [dist,p1,p2] = linesegtopoly(testobj,poly);
         if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
      end
    end
%      for i=1:5
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%          plot3(sur_x, sur_y,sur_z,'-r','LineWidth',2);
%          hold on;
% plot3(testobj(:,1),testobj(:,2),testobj(:,3),'-b','LineWidth',3)
% plot3([po1(1,1),po2(1,1)],[po1(1,2),po2(1,2)],[po1(1,3),po2(1,3)],'-b','LineWidth',3)
 %    end
    if option == 3
        [dist,p1,p2] = pointtopoly(testobj,obsp);
        if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
    end
    if option == 4
       [dist,p1,p2] = linesegtopoly(testobj,obsp);
       if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
       end
    end
%       plot3(testobj(:,1),testobj(:,2),testobj(:,3),'-b','LineWidth',3);
%     hold on;
%     plot3(obsp(:,1),obsp(:,2),obsp(:,3),'-r','LineWidth',3);
%      plot3([po1(1,1),po2(1,1)],[po1(1,2),po2(1,2)],[po1(1,3),po2(1,3)],'-b','LineWidth',3)
if option == 5
    num_bor = size(testobj,1);
    for i = 1:num_bor
        if i == num_bor
            bor = [testobj(num_bor,:);testobj(1,:)];
        else
            bor = [testobj(i,:);testobj(i+1,:)];
        end
        [dist,p1,p2] = linesegtopoly(bor,obsp);
        if dist < dis 
            dis = dist;
            po1 = p1;
            po2 = p2;
       end
    end
end
if option == 6
    num_bor  = size(testobj,1);
     [obs_sur] =obp_tran(obsp);
    for i=1:5
        num_p = size(obs_sur,2)/3;
        for j=1:num_p
        poly(j,:) = obs_sur(i,j*3-2:j*3,1);
        end
        [dist,p1,p2] =dis_compute(testobj,poly,5);
        if dist < dis
            dis = dist;
            po1 = p1;
            po2 = p2;
        end
    end   
    
%  fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
%  patch('Faces',fac_obs,'Vertices',obsp,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',0.3);
%          hold on;
% fac = [1 2 3];
%  patch('Faces',fac,'Vertices',testobj,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',0.3);
%   plot3([po1(1,1),po2(1,1)],[po1(1,2),po2(1,2)],[po1(1,3),po2(1,3)],'-b','LineWidth',3);
end
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
if self.draw_flag
    for i = 1: num_ob                                              %需要把障碍物画出来时则加上此段
     fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
     patch('Faces',fac_obs,'Vertices',obp(i*8-7:i*8,:)*1000,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',1);

             hold on;
    end
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

function [dis,po1,po2] = linesegtopoly(lineseg,poly)%线段与面求距,返回最短距离与相应的两点,po1为线段上的点，po2为面上的点
num_p = size(poly,1);
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
lin = lineseg(2,:)-lineseg(1,:);            %先求解线段是否与面相交，相交距离为0；      
kk = dot(vec,poly(1,:)-lineseg(1,:))/dot(vec,lineseg(2,:)-lineseg(1,:)); 
interp = lineseg(1,:) + lin*kk;
state1 = insidepoly3(interp,poly);
if kk >= 0&&kk <= 1&&state1 == 1
    dis = 0;
    po1 = interp;
    po2 = interp;
    return;
end
[state,pro1] = point_pro_poly(lineseg(1,:),poly);    %求线段端点到面的最近距离
if state ==1
lin1 = norm(lineseg(1,:)-pro1);
else 
    lin1 = 10000;
end
[state,pro2] = point_pro_poly(lineseg(2,:),poly);
if state ==1
lin2 = norm(lineseg(2,:)-pro2);
else 
    lin2 = 10000;
end
%dis = min([lin1,lin2]);
if lin1<lin2
    dis = lin1;
    po1 = lineseg(1,:);
    po2 = pro1;
else
    dis = lin2;
    po1 = lineseg(2,:);
    po2 = pro2;
end
for i=1:num_p                    %求线段与线段间的最近距离
    if i == num_p
        seg = [poly(num_p,:);poly(1,:)];
       [dist,sp1,sp2] = Seg2SegDist(lineseg,seg);
    else
        seg = [poly(i,:);poly(i+1,:)];
        [dist,sp1,sp2] = Seg2SegDist(lineseg,seg);
    end
    if dist < dis
        dis = dist;
        po1 = sp1;
        po2 = sp2;
    end
end
end

function [dis,po1,po2] = pointtopoly(poi,poly)       %点与面求距,返回最短距离与相应的两点,po1原来的点，po2为面上的点
 state1 =  insidepoly3(poi,poly);
 num_p = size(poly,1);
 if state1 == 1
     dis = 0;
     po1 = poi;
     po2 = poi;
     return;
 end
 [state,pro] = point_pro_poly(poi,poly);  %先求点在面上的投影点及距离
 if state ==1
     dis = norm (poi-pro);
     po1 = poi;
     po2 = pro;
 else
     dis = 10000;
     po1 = poi;
     po2 = pro;
 end
 for i= 1:num_p                   %求点与面上边的最小距离
     if i == num_p
         line = [poly(num_p,:);poly(1,:)];
     else
         line = [poly(i,:);poly(i+1,:)];
     end
     [dist,lp] = point_to_line(poi,line);
     if dist < dis
         dis = dist;
         po1 = poi;
         po2 = lp;
     end
 end
end

function [dis,lp] = point_to_line(poi,line)   %求点到线段的最小距离，返回距离及对应的点
   if norm(line(1,:) - line(2,:))<1e-6
       dis = norm(line(1,:) - poi);
       lp = line(1,:);
       return;
   end
   vk = line(2,:) - line(1,:);
   v1 = poi - line(1,:);
   v2 = poi - line(2,:);
   if dot(vk,v1) < 1e-6
       lp = line(1,:);
       dis = norm(v1);
       return;
   end
   if dot(vk,v2) > 1e-6
       lp = line(2,:);
       dis = norm(v2);
       return;
   end
       dis = norm(cross(vk,v1)/norm(vk));
       lp = line(1,:) + vk*(dot(vk,v1)/dot(vk,vk));  
end
 
function [state,pro] = point_pro_poly(point,poly)      %点在多边形内部的投影
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
vec = vec/norm(vec);
prop = point-vec*dot(point-poly(1,:),vec);
if insidepoly3(prop,poly)==1
    state = 1;
    pro = prop;
else
    state = 0;
    pro = 0;
end
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

function [ T,joi_p ] =  Kine5D( joint_ang ,option)
%通过正运动学求机器人关节点的位姿,输入关节角，输出关节的位姿,根据option的大小来求解不同关节点相对于基座标的位姿
%global L1 L2;
joint_ang = deg2rad(joint_ang);
joi = joint_ang;

Len=[335,293.2,293.2,335];
%Len=[L1,L2,L2,L1]*1000;
T_1b = [cos(joi(1)) -sin(joi(1)) 0 0;sin(joi(1)) cos(joi(1)) 0 0;0 0 1 0;0 0 0 1];
T_21 = [cos(joi(2)) -sin(joi(2)) 0 0;0 0 -1 0;sin(joi(2)) cos(joi(2)) 0 Len(1);0 0 0 1];
T_32 = [cos(joi(3)) -sin(joi(3)) 0 Len(2);sin(joi(3)) cos(joi(3)) 0 0;0 0 1 0;0 0 0 1];
T_43 = [cos(joi(4)) -sin(joi(4)) 0 Len(3);sin(joi(4)) cos(joi(4)) 0 0; 0 0 1 0; 0 0 0 1];
T_54 = [cos(joi(5)) -sin(joi(5)) 0 0;0 0 -1 -Len(4);sin(joi(5)) cos(joi(5)) 0 0;0 0 0 1];
if option == 1%T_b_2
    T = T_1b*T_21;
    joi_p = T(1:3,4)';
elseif option == 2%T_b_3
    T = T_1b*T_21*T_32;
    joi_p = T(1:3,4)';
elseif option == 3%T_b_4
    T = T_1b*T_21*T_32*T_43;
    joi_p = T(1:3,4)';
elseif option == 4%T_b_5
    T = T_1b*T_21*T_32*T_43*T_54;
    joi_p = T(1:3,4)';
else
     h=msgbox('运动学求解失败','warn'); 
     uiwait(h,3);
     return;
end

end

% ==========================================================
% 函数功能：根据机器人末端位姿齐次矩阵（在机器人基座标系下）求解机器人关节角
%           强制机器人的末端满足构型约束
% 输入： EETrMtx - 机器人末端位姿齐次矩阵
%       CurConfigs - 机器人当前的关节角，5维
%	jpos_G2[0] = - gdJPos[4];
%	jpos_G2[1] = gdJPos[3];
%	jpos_G2[2] = gdJPos[2];
%	jpos_G2[3] = gdJPos[1];
%	jpos_G2[4] = -gdJPos[0];
%
% 输出： Configs - 表示机器人5个关节角的向量
function [Flag, Configs] = IKine5DNew(robot)

%Q:怎么规定连杆序号的？
%A:反正对称,应该base是1
l01 = robot.LinksLen(1);
l2 = robot.LinksLen(2);
l3 = robot.LinksLen(3);
l45 = robot.LinksLen(4);

CurConfigs = deg2rad(robot.CurJAs);
% 四组解均存在的情况，1为存在
Sol_Flag = ones(1,4);
Configs = NaN * ones(1,5);  % 最后解输出
theta = ones(4,5);     % 4组逆解的暂存空间

% 不同夹持端处理（加了个断点，发现不会进else？）
if robot.FixedGripper == 1
    EETrMtx = robot.TarGraspMtx;
else
    Trx = [1 0 0 0;0 -1 0 0; 0 0 -1 0; 0 0 0 1];
    EETrMtx = Trx / robot.TarGraspMtx * Trx;
end

% 先求解theta1
% 求解theta1
theta(1:2,1) = ClampJAngle(CurConfigs(1),atan2(EETrMtx(2,4),EETrMtx(1,4)));
% 另一解
theta(3:4,1) = ClampJAngle(CurConfigs(1),atan2(-EETrMtx(2,4),-EETrMtx(1,4)));

% 测试夹持方向是否在机器人构型平面内（吸附不用判断）
vdot = dot(EETrMtx(1:3,3),[-EETrMtx(2,4),EETrMtx(1,4),0]);
% if abs(vdot) > 1e-12
%     EETrMtx(1:3,1:3) = modifyOriMtx(EETrMtx(1:3,4),EETrMtx(1:3,1));
% end

% 变换到腕关节求解
EETrMtx(1:3,4) = EETrMtx(1:3,4) - EETrMtx(1:3,3) * l45;

% 求解theta5 和 theta3
for i = 1:2
    
    s1 = sin(theta(2*i-1,1));
    c1 = cos(theta(2*i-1,1));

    % 注意 si表示从角度直接求出的正弦值，si_表示从位姿矩阵间接求出的正弦值
    s5_ = s1 * EETrMtx(1,1) - c1 * EETrMtx(2,1);
    c5_ = s1 * EETrMtx(1,2) - c1 * EETrMtx(2,2);
    
    % theta5
    theta(2*i-1:2*i,5) = ClampJAngle(CurConfigs(5),atan2(s5_,c5_));
    
    % theta3
    c3_ = ((EETrMtx(1,4)*c1 + EETrMtx(2,4)*s1)^2 + (EETrMtx(3,4) - l01)^2 - (l2^2+l3^2))/(2*l2*l3);
    s3_sq = 1 - c3_^2;
    
    if (abs(s3_sq) < 1e-12)   %等于零
        s3_ = 0; 
    elseif (s3_sq < 0)       %小于零，无解
        Sol_Flag(2*i-1:2*i) = 0;
        continue;
    else
        s3_ = sqrt(s3_sq);
    end
    
    theta(2*i-1,3) = ClampJAngle(CurConfigs(3),atan2(s3_,c3_));
    % 另一种解,怎么把两个值赋给同一个内存空间！！
    theta(2*i,3) = ClampJAngle(CurConfigs(3),atan2(-s3_,c3_));
    
end

if ~Sol_Flag(1) && ~Sol_Flag(3)   %腰关节确定无解
    Flag = 0;
    return;
end
    
% 求解theta2 和 theta4
for i = 1:4
    
    if Sol_Flag(i)
        
        s1 = sin(theta(i,1));
        c1 = cos(theta(i,1));
        
        s3 = sin(theta(i,3));
        c3 = cos(theta(i,3));
        
        % theta2
        pzl01 =  EETrMtx(3,4) - l01;
        pxpy = EETrMtx(1,4) * c1 + EETrMtx(2,4) * s1;        
        s2_ = pzl01 * (l3 * c3 + l2) - pxpy * l3 * s3;  % 分母相等且为正，可都不计算
        c2_ = pxpy * (l3 * c3 + l2) + pzl01 * l3 * s3;  % 分母相等且为正，可都不计算
        
        theta(i,2) = ClampJAngle(CurConfigs(2),atan2(s2_,c2_));
        
        % theta4
        s234_ = EETrMtx(1,3) * c1 + EETrMtx(2,3) * s1;
        c234_ = - EETrMtx(3,3);
        
        theta234 = atan2(s234_,c234_);  %234之和
        theta(i,4) = ClampJAngle(CurConfigs(4), theta234 - theta(i,2) - theta(i,3));
        
    end
end

% 关节角限位
JiontLimitUp = deg2rad([360,240,140,240,360]);
JiontLimitLow = deg2rad([-360,-60,-140,-60,-360]);

% 选择最佳结果
criterion = ones(1,4);      %可以用inf*ones(1,4)进一步防止错误出现
result = 0;

for i = 1:4
    
    if Sol_Flag(i)       
        % 是否为Nan元素
        if find(isnan(theta(i,:))==1,1)
            Sol_Flag(i) = 0;
            continue;
        end
    end
    if Sol_Flag(i)   % 有解的才进行关节角检查
        % 是否超出限位
        if any(theta(i,:) > JiontLimitUp) || any(theta(i,:) < JiontLimitLow)      % 超出限位
            Sol_Flag(i) = 0;
            continue; % 提前结束限位检查
        end
    end
    if Sol_Flag(i)
        % 这个度量函数不是最合适的？
        criterion(i) = abs(theta(i,1)-CurConfigs(1)) + ...
            abs(theta(i,2)-CurConfigs(2)) + ...
            abs(theta(i,3)-CurConfigs(3)) + ...
            abs(theta(i,4)-CurConfigs(4));
            %abs(theta(i,5)-CurConfigs(5));
        if ~result  %第一次得到有效解，先进行初始化
            result = i;
            best = criterion(i);
        elseif best > criterion(i)      %第二次以及以后，对有效解进行比较
            result = i;                 %更优的话就替换掉原有解
            best = criterion(i);
        elseif abs(best - criterion(i))<1e-5
            if theta(i,2)>0        
                 result = i;                 %偏向往上凸的构型，即第二个摆转关节逆向摆
            best = criterion(i);
            else
               continue; 
            end
        end
    end
    
end

if result
    Flag = 1;
    Configs(1,:) = rad2deg(theta(result,:));     %选择最优的解进行输出
else
    Flag = 0;
end

end

% 根据当前角度优化计算转角，限定在[-360,+360]内
function betterJA = ClampJAngle(CurJA,TarJA)

dPI = 2*pi;

if TarJA > dPI
    betterJA = TarJA - floor(TarJA/dPI)*dPI;
elseif TarJA < - dPI
    TarJA = TarJA + dPI;
    betterJA = TarJA - floor(TarJA/dPI)*dPI-dPI;
elseif TarJA >= 0
    if (abs(TarJA - CurJA) > abs(TarJA - CurJA - dPI))
        betterJA = TarJA - dPI;
    else
        betterJA = TarJA;
    end
else
    if (abs(TarJA - CurJA) > abs(TarJA - CurJA + dPI))
        betterJA = TarJA + dPI;
    else
        betterJA = TarJA;
    end
end

end

% 修改夹持方向，使5DoFs Climbot能到达
% 也可以理解为以保证位置和杆向（姿态矩阵的n列）为根本
function OriMtx = modifyOriMtx(Pos,n)

% OriMtx = [n|o|a]

% 计算夹持方向
if abs(n(3)) > 1e-12
    alpha = atan2(Pos(2),Pos(1));
    tanA = tan(alpha);
    lambda = -(n(1)+n(2)*tanA)/n(3);
    a = n; % 初始化
    if Pos(1) > 0
        a(1) = sqrt(1 / (1 + tanA^2 + lambda^2));
    else
        a(1) = -sqrt(1 / (1 + tanA^2 + lambda^2));
    end
    a(2) = a(1) * tanA;
    a(3) = lambda * a(1);
else
    a = [0;0;-1];
end
o = cross(a,n);

OriMtx = [n,o,a];

end

function[state,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,eff_p,enp)   %计算机器人本体斥力向量和机器人末端斥力向量和目标引力对机器人运动的影响，输入本体最小距离和末端最小距离，当前末端点和目标点
   vec_re = zeros(1,3);
    dis_warn = 0.05;  %注意距离为5cm
  dis_safe = 0.02;  %最小安全距离为2cm
  vec_lin = vec_lin/norm(vec_lin);
  vec_eff = vec_eff/norm(vec_eff);
  vec_tar = (enp - eff_p)/norm(enp - eff_p);
     if dis1 <= 0.003 || dis2 <= 0.003
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

%% ==========================================================
% 函数功能：根据机器人末端位姿齐次矩阵（在机器人基座标系下）求解机器人关节角
%           强制机器人的末端满足构型约束
% 输入： EETrMtx - 机器人末端位姿齐次矩阵
%       CurConfigs - 机器人当前的关节角，5维
%	jpos_G2[0] = - gdJPos[4];
%	jpos_G2[1] = gdJPos[3];
%	jpos_G2[2] = gdJPos[2];
%	jpos_G2[3] = gdJPos[1];
%	jpos_G2[4] = -gdJPos[0];
%
% 输出： Configs - 表示机器人5个关节角的向量,求次优解
function [Flag, Configs] = IKine5D22(robot)

l01 = robot.LinksLen(1);
l2 = robot.LinksLen(2);
l3 = robot.LinksLen(3);
l45 = robot.LinksLen(4);

CurConfigs = deg2rad(robot.CurJAs);
% 四组解均存在的情况，1为存在
Sol_Flag = ones(1,4);
Configs = NaN * ones(1,5);  % 最后解输出
theta = ones(4,5);     % 4组逆解的暂存空间

% 不同夹持端处理
if robot.FixedGripper == 1
    EETrMtx = robot.TarGraspMtx;
else
    Trx = [1 0 0 0;0 -1 0 0; 0 0 -1 0; 0 0 0 1];
    EETrMtx = Trx / robot.TarGraspMtx * Trx;
end

% 先求解theta1
% 求解theta1
theta(1:2,1) = ClampJAngle(CurConfigs(1),atan2(EETrMtx(2,4),EETrMtx(1,4)));
% 另一解
theta(3:4,1) = ClampJAngle(CurConfigs(1),atan2(-EETrMtx(2,4),-EETrMtx(1,4)));

% 测试夹持方向是否在机器人构型平面内
vdot = dot(EETrMtx(1:3,3),[-EETrMtx(2,4),EETrMtx(1,4),0]);
% if abs(vdot) > 1e-12
%     EETrMtx(1:3,1:3) = modifyOriMtx(EETrMtx(1:3,4),EETrMtx(1:3,1));
% end

% 变换到腕关节求解
EETrMtx(1:3,4) = EETrMtx(1:3,4) - EETrMtx(1:3,3) * l45;

% 求解theta5 和 theta3
for i = 1:2
    
    s1 = sin(theta(2*i-1,1));
    c1 = cos(theta(2*i-1,1));

    % 注意 si表示从角度直接求出的正弦值，si_表示从位姿矩阵间接求出的正弦值
    s5_ = s1 * EETrMtx(1,1) - c1 * EETrMtx(2,1);
    c5_ = s1 * EETrMtx(1,2) - c1 * EETrMtx(2,2);
    
    % theta5
    theta(2*i-1:2*i,5) = ClampJAngle(CurConfigs(5),atan2(s5_,c5_));
    
    % theta3
    c3_ = ((EETrMtx(1,4)*c1 + EETrMtx(2,4)*s1)^2 + (EETrMtx(3,4) - l01)^2 - (l2^2+l3^2))/(2*l2*l3);
    s3_sq = 1 - c3_^2;
    
    if (abs(s3_sq) < 1e-12)   %等于零
        s3_ = 0; 
    elseif (s3_sq < 0)       %小于零，无解
        Sol_Flag(2*i-1:2*i) = 0;
        continue;
    else
        s3_ = sqrt(s3_sq);
    end
    
    theta(2*i-1,3) = ClampJAngle(CurConfigs(3),atan2(s3_,c3_));
     if theta(2*i-1,3) <-pi
         theta(2*i-1,3) = theta(2*i-1,3) +2*pi;
     elseif  theta(2*i-1,3) >pi
         theta(2*i-1,3) = theta(2*i-1,3) -2*pi;
     end
    % 另一种解,怎么把两个值赋给同一个内存空间！！
    theta(2*i,3) = ClampJAngle(CurConfigs(3),atan2(-s3_,c3_));
    if theta(2*i,3) <-pi
         theta(2*i,3) = theta(2*i-1,3) +2*pi;
     elseif  theta(2*i,3) >pi
         theta(2*i,3) = theta(2*i-1,3) -2*pi;
     end
end

if ~Sol_Flag(1) && ~Sol_Flag(3)   %腰关节确定无解
    Flag = 0;
    return;
end
    
% 求解theta2 和 theta4
for i = 1:4
    
    if Sol_Flag(i)
        
        s1 = sin(theta(i,1));
        c1 = cos(theta(i,1));
        
        s3 = sin(theta(i,3));
        c3 = cos(theta(i,3));
        
        % theta2
        pzl01 =  EETrMtx(3,4) - l01;
        pxpy = EETrMtx(1,4) * c1 + EETrMtx(2,4) * s1;        
        s2_ = pzl01 * (l3 * c3 + l2) - pxpy * l3 * s3;  % 分母相等且为正，可都不计算
        c2_ = pxpy * (l3 * c3 + l2) + pzl01 * l3 * s3;  % 分母相等且为正，可都不计算
        
        theta(i,2) = ClampJAngle(CurConfigs(2),atan2(s2_,c2_));
        
        % theta4
        s234_ = EETrMtx(1,3) * c1 + EETrMtx(2,3) * s1;
        c234_ = - EETrMtx(3,3);
        
        theta234 = atan2(s234_,c234_);  %234之和
        theta(i,4) = ClampJAngle(CurConfigs(4), theta234 - theta(i,2) - theta(i,3));
        
    end
end

% 关节角限位
JiontLimitUp = deg2rad([360,230,140,230,360]);
JiontLimitLow = deg2rad([-360,-50,-140,-50,-360]);

% 选择最佳结果
criterion = ones(1,4)*10000;      %可以用inf*ones(1,4)进一步防止错误出现
result = 0;

for i = 1:4
    
    if Sol_Flag(i)       
        % 是否为Nan元素
        if find(isnan(theta(i,:))==1,1)
            Sol_Flag(i) = 0;
            continue;
        end
    end
    if Sol_Flag(i)   % 有解的才进行关节角检查
        % 是否超出限位
        if any(theta(i,:) > JiontLimitUp) || any(theta(i,:) < JiontLimitLow)      % 超出限位
            Sol_Flag(i) = 0;
            continue; % 提前结束限位检查
        end
    end
    if Sol_Flag(i)==1
        joi = rad2deg(theta(i,:));
     
        % 这个度量函数不是最合适的？
        criterion(i) = abs(theta(i,1)-CurConfigs(1)) + ...
            abs(theta(i,2)-CurConfigs(2)) + ...
            abs(theta(i,3)-CurConfigs(3)) + ...
            abs(theta(i,4)-CurConfigs(4));
            %abs(theta(i,5)-CurConfigs(5));
        if ~result  %第一次得到有效解，先进行初始化
            result = i;
            best = criterion(i);
        elseif best > criterion(i)      %第二次以及以后，对有效解进行比较
            result = i;                 %更优的话就替换掉原有解
            best = criterion(i);
        elseif abs(best - criterion(i))<1e-5
            if theta(i,2)>0        
                 result = i;                 %偏向往上凸的构型，即第二个摆转关节逆向摆
            best = criterion(i);
            else
               continue; 
            end
        end
 
    end
    
end

% if result>0
%     [min_ang,num_ang] = min(criterion);
%     criterion(num_ang) = 10000;
%     [min_ang,num_ang2] = min(criterion);
%     if Sol_Flag(num_ang2)==1
%     Flag = 1;
%     Configs(1,:) = rad2deg(theta(num_ang2,:));     %选择次优的解进行输出
%     else
%         Flag = 0;
%     end
% else
%     Flag = 0;
% end

if result
    Flag = 1;
    Configs(1,:) = rad2deg(theta(result,:));     %选择最优的解进行输出
    Configs(1,1) = Configs(1,1)+180;
    if Configs(1,1)>180
        Configs(1,1) = Configs(1,1)-360;
    elseif Configs(1,1)<-180
        Configs(1,1) = Configs(1,1)+360;
    end
    Configs(1,2) =180-Configs(1,2);
    Configs(1,3) =-Configs(1,3);
    Configs(1,4) =180-Configs(1,4);
else
    Flag = 0;
end

end