function [] = motion_simulation(Ba,Ba_R,Ang,filename)  %��·���滮��������棬�������ɶ���
            nargin = 2;
            if nargin > 1 && ischar(filename)  % Ҫ����AVI�ļ�
                saveFlag = 1;
                % avi����
                aviobj = VideoWriter(filename);
                open(aviobj);
                close all;
            else
                saveFlag = 0;
            end
                        draw_obstruct
            
            % ��ջ����˵ľ�����Ա�֤�ܶ�λ�ͼ
          hRobot = [];
            
            % ����泡��
           % DrawTruss(self.truss,1);
            
            % ��������
         %   camlight headlight;         % ����յ�� 
       %     camlight headlight;
            material metal              % ��������
%             % view(3)                     % ��άͶӰ�ӽ�

%             axis equal;                 % ��������

%          
%             
%             % 9���˳���
%            % view(-23,20);
%             view(-9,-1); 
%             view(-187,-39);
%             view(-88,21);
             view(-150,-320);
%          %   view(-44,13);
%            % axis([0,2000,-500,2600,-150,2600]); 
            path_co = 2;
%             % 25���˳���
%             view(294.8,11);
%             axis([0,6000,0,6000,0,6000]);     % ���С25���˼�ʱ
            %==================================================================
            plot_surface2;
            [map_border, obstruct_border, obstruct_height] = input_map2;
             plot_obs(map_border,obstruct_border,obstruct_height,eye(3),[0 0 0]);
%              axis([0,8500,0,7000,0,1000]);
%               axis([-3000,2500,-3000,3000,0,3000]);
axis([0,3000,-1500,0, 0,2500])
%             fz = 16;                    % �ֺ�
%             xlabel('\it{x}\rm(\it{mm}\rm)','FontSize',fz,'FontName','Times New Roman');
%             ylabel('\it{y}\rm(\it{mm}\rm)','Fontsize',fz,'FontName','Times New Roman');
%             zlabel('\it{z}\rm(\it{mm}\rm)','Fontsize',fz,'FontName','Times New Roman');
%             set(gca,'fontsize',fz);
%             set(gca,'Fontname','Times New Roman');
          set(gcf,'Units','inches','Position',[2 1 13 5]);  % ����λ�úʹ�С
           set(gcf,'doublebuffer','on')                             % ��������
            
            % �ٻ�������
%             drobot = self.robot;        % ר�����ڻ�ͼ
             color = 'g';
               CurHMatrix = eye(4);
            drobot = struct('DoFs',5,'LinksLen',[329,293.2,293.2,329],'ModuleRadii',50,...
                        'CurGraspMtx',CurHMatrix,'CurJAs',zeros(1,5),'FixedGripper',1,'TarGraspMtx',eye(4,4),...
                          'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
            
            stepsNum = size(Ang,2);   % �ܵĲ���
%               aStep =Ang{1};  % ȡ����k��������
%                 drobot.CurGraspMtx(1:3,1:3) = Ba_R{1};
%                 drobot.CurGraspMtx(1:3,4) = Ba(1,:)'*1000;
%                 drobot.TarJAs = aStep(1,:);
%              hRobot = DrawRobotmo(drobot,1,1);
%                aStep =Ang{stepsNum};  % ȡ����k��������
%                 drobot.CurGraspMtx(1:3,1:3) = Ba_R{stepsNum};
%                 drobot.CurGraspMtx(1:3,4) = Ba(stepsNum,:)'*1000;
%                 drobot.TarJAs = aStep(size(aStep,1),:);
%              hRobot = DrawRobotmo(drobot,1,1);
             
             
            for k = 1:stepsNum    % k��ʾÿһ��
                
              %  disp(['��ǰ���ڲ��ŵ� ',num2str(k),' �������˶�������']);
                
                aStep =Ang{k};  % ȡ����k��������
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
                % �������˹����г����������֤�����������һ��
%                 if k == 1
%                     drobot.FixedGripper = self.footholds{self.solIndex}(k).gripper;
%                 else
%                     if drobot.FixedGripper == 1
%                         drobot.FixedGripper = 2;
%                     else
%                         drobot.FixedGripper = 1;
%                     end
%                 end
%                 disp(['fa���ŵļг�����',num2str(self.footholds{self.solIndex}(k).gripper)]);
%                 disp(['�˹����ŵļг�����',num2str(drobot.FixedGripper)]);
              %  iState = state(drobot.DoFs,aStep(1).s);        % ��һ���ڵ�
                % Ҫ����Ŀ��гֵ�ľ��󣬷���FixedGripper=2��ʱ��ͼ�����
               % drobot.TarGraspMtx = iState.toTMatrix();
               
                drobot.TarJAs = aStep(1,:);
             %   ��ͼ��������㲿���ľ��
                if isempty(hRobot)
                    hRobot = DrawRobotmo(drobot,1,1);
                else
                    MoveRobot(hRobot,drobot);
                end
                
%                 if saveFlag              %¼�ƶ���ʱ�ã���ֻ�۲����Ч������¼�ƶ���ʱ��ע�͵�
%                     Frame = getframe(gcf);
%                    % Frame = getframe;
%                     writeVideo(aviobj,Frame);
%                 end
              
                % ���µ�ǰ�ؽڽǶ�
                drobot.CurJAs = drobot.TarJAs;
                
                % ����Ľڵ�������ʾ
                Nnodes = length(aStep);
               % lastPos = hTransformVertices(drobot.TarGraspMtx(1:3,4)',drobot.CurGraspMtx);
                for i = 2:Nnodes
                   % temState = state(drobot.DoFs,aStep(i).s);
                  %  drobot.TarGraspMtx = temState.toTMatrix();
                   
                    % ˳����ȡ�滮�Ĺؽڽǣ������������⣬�Ա�֤�������˼�
                    % gu
                    % ================================================= 
                    drobot.TarJAs = aStep(i,:);
                    % ��ͼ��������㲿���ľ��
%                     self.hRobot = DrawRobot(drobot,1,1);
                    MoveRobot(hRobot,drobot,1);
                     [ T,joi_p1 ] =  Kine5D( aStep(i,:) ,4);
                     joi_p1 = (Ba_R{k}*joi_p1')'+Ba(k,:)*1000;
                     
                     %%%%%%%%%%%���켣
%                     if path_co ==1
%                      plot3(joi_p1(:,1), joi_p1(:,2),joi_p1(:,3),'.m','LineWidth',0.1);
%                     else
%                          plot3(joi_p1(:,1), joi_p1(:,2),joi_p1(:,3),'.k','LineWidth',0.1);
%                     end
                    % ���µ�ǰ�ؽڽǶ�
                    drobot.CurJAs = drobot.TarJAs;
                    % ================================================= 
                    
                   
                  
                    if saveFlag              %¼�ƶ���ʱ��
                        Frame = getframe(gcf);
                        %Frame = getframe;
                        writeVideo(aviobj,Frame);
                    end
                  end
%                 pause(0.1);  % ÿһ���г��Ϻ���ʱ1s
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
