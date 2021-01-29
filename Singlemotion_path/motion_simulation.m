function [] = motion_simulation(Ba,Ba_R,Ang,filename)  %将路径规划结果做仿真，并且生成动画
            nargin = 2;
            if nargin > 1 && ischar(filename)  % 要保留AVI文件
                saveFlag = 1;
                % avi对象
                aviobj = VideoWriter(filename);
                open(aviobj);
                close all;
            else
                saveFlag = 0;
            end
                        draw_obstruct
            
            % 清空机器人的句柄，以保证能多次画图
          hRobot = [];
            
            % 先描绘场景
           % DrawTruss(self.truss,1);
            
            % 窗口设置
         %   camlight headlight;         % 开两盏灯 
       %     camlight headlight;
            material metal              % 金属材质
%             % view(3)                     % 三维投影视角

%             axis equal;                 % 轴比例相等

%          
%             
%             % 9根杆场景
%            % view(-23,20);
%             view(-9,-1); 
%             view(-187,-39);
%             view(-88,21);
             view(-150,-320);
%          %   view(-44,13);
%            % axis([0,2000,-500,2600,-150,2600]); 
            path_co = 2;
%             % 25根杆场景
%             view(294.8,11);
%             axis([0,6000,0,6000,0,6000]);     % 轴大小25根杆件时
            %==================================================================
            plot_surface2;
            [map_border, obstruct_border, obstruct_height] = input_map2;
             plot_obs(map_border,obstruct_border,obstruct_height,eye(3),[0 0 0]);
%              axis([0,8500,0,7000,0,1000]);
%               axis([-3000,2500,-3000,3000,0,3000]);
axis([0,3000,-1500,0, 0,2500])
%             fz = 16;                    % 字号
%             xlabel('\it{x}\rm(\it{mm}\rm)','FontSize',fz,'FontName','Times New Roman');
%             ylabel('\it{y}\rm(\it{mm}\rm)','Fontsize',fz,'FontName','Times New Roman');
%             zlabel('\it{z}\rm(\it{mm}\rm)','Fontsize',fz,'FontName','Times New Roman');
%             set(gca,'fontsize',fz);
%             set(gca,'Fontname','Times New Roman');
          set(gcf,'Units','inches','Position',[2 1 13 5]);  % 窗口位置和大小
           set(gcf,'doublebuffer','on')                             % 消除抖动
            
            % 再画机器人
%             drobot = self.robot;        % 专门用于画图
             color = 'g';
               CurHMatrix = eye(4);
            drobot = struct('DoFs',5,'LinksLen',[329,293.2,293.2,329],'ModuleRadii',50,...
                        'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
                          'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
            
            stepsNum = size(Ang,2);   % 总的步数
%               aStep =Ang{1};  % 取出第k步的数据
%                 drobot.CurGraspMtx(1:3,1:3) = Ba_R{1};
%                 drobot.CurGraspMtx(1:3,4) = Ba(1,:)'*1000;
%                 drobot.TarJAs = aStep(1,:);
%              hRobot = DrawRobotmo(drobot,1,1);
%                aStep =Ang{stepsNum};  % 取出第k步的数据
%                 drobot.CurGraspMtx(1:3,1:3) = Ba_R{stepsNum};
%                 drobot.CurGraspMtx(1:3,4) = Ba(stepsNum,:)'*1000;
%                 drobot.TarJAs = aStep(size(aStep,1),:);
%              hRobot = DrawRobotmo(drobot,1,1);
             
             
            for k = 1:stepsNum    % k表示每一步
                
              %  disp(['当前正在播放第 ',num2str(k),' 步攀爬运动动画。']);
                
                aStep =Ang{k};  % 取出第k步的数据
                drobot.CurGraspMtx(1:3,1:3) = Ba_R{k};
                drobot.CurGraspMtx(1:3,4) = Ba(k,:)'*1000;
                 if path_co ==2
                        path_co = 1;
                        drobot.FixedGripper = 2;
                    else
                        path_co = 2;
                        drobot.FixedGripper = 1;
                 end
         %        drobot.FixedGripper = 1;
                % 还不如人工换夹持器，充分验证后可以用下面一句
%                 if k == 1
%                     drobot.FixedGripper = self.footholds{self.solIndex}(k).gripper;
%                 else
%                     if drobot.FixedGripper == 1
%                         drobot.FixedGripper = 2;
%                     else
%                         drobot.FixedGripper = 1;
%                     end
%                 end
%                 disp(['fa安排的夹持器：',num2str(self.footholds{self.solIndex}(k).gripper)]);
%                 disp(['人工安排的夹持器：',num2str(drobot.FixedGripper)]);
              %  iState = state(drobot.DoFs,aStep(1).s);        % 第一个节点
                % 要读入目标夹持点的矩阵，否则FixedGripper=2的时候画图会出错
               % drobot.TarGraspMtx = iState.toTMatrix();
               
                drobot.TarJAs = aStep(1,:);
             %   画图并保存各零部件的句柄
                if isempty(hRobot)
                    hRobot = DrawRobotmo(drobot,1,1);
                else
                    MoveRobot(hRobot,drobot);
                end
                
%                 if saveFlag              %录制动画时用，若只观察仿真效果，不录制动画时可注释掉
%                     Frame = getframe(gcf);
%                    % Frame = getframe;
%                     writeVideo(aviobj,Frame);
%                 end
              
                % 更新当前关节角度
                drobot.CurJAs = drobot.TarJAs;
                
                % 后面的节点连续显示
                Nnodes = length(aStep);
               % lastPos = hTransformVertices(drobot.TarGraspMtx(1:3,4)',drobot.CurGraspMtx);
                for i = 2:Nnodes
                   % temState = state(drobot.DoFs,aStep(i).s);
                  %  drobot.TarGraspMtx = temState.toTMatrix();
                   
                    % 顺序提取规划的关节角，而不再求解逆解，以保证不会干涉杆件
                    % gu
                    % ================================================= 
                    drobot.TarJAs = aStep(i,:);
                    % 画图并保存各零部件的句柄
%                     self.hRobot = DrawRobot(drobot,1,1);
                    MoveRobot(hRobot,drobot,1);
                     [ T,joi_p1 ] =  Kine5D( aStep(i,:) ,4);
                     joi_p1 = (Ba_R{k}*joi_p1')'+Ba(k,:)*1000;
                     
                     %%%%%%%%%%%画轨迹
%                     if path_co ==1
%                      plot3(joi_p1(:,1), joi_p1(:,2),joi_p1(:,3),'.m','LineWidth',0.1);
%                     else
%                          plot3(joi_p1(:,1), joi_p1(:,2),joi_p1(:,3),'.k','LineWidth',0.1);
%                     end
                    % 更新当前关节角度
                    drobot.CurJAs = drobot.TarJAs;
                    % ================================================= 
                    
                   
                  
                    if saveFlag              %录制动画时用
                        Frame = getframe(gcf);
                        %Frame = getframe;
                        writeVideo(aviobj,Frame);
                    end
                  end
%                 pause(0.1);  % 每一步夹持上后延时1s
                if strcmp(color,'g')
                    color = 'r';
                else
                    color = 'g';
                end
            end
           
            if saveFlag
                close(aviobj);
            end
            
        end
