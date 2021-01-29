% function [  ] = jointangle_transform( JJ )
% %将关节角数据转换成攀爬机器人离线运动代码
% CREATE_G_CODE = 1;
% DISTANCE2WALL_STEP_NUM = 9;%沿壁面方向量上抬的步数
% filename = strcat('G_CODE_',strrep(datestr(now,26),'/','_'));
% filename = strcat(filename,'_');
% filename = strcat(filename,strrep(datestr(now,13),':','_'));
% filename = strcat(filename,'_');
% filename = strcat(filename,num2str(DISTANCE2WALL_STEP_NUM));
% filename = strcat(filename,'cm');
% filename = strcat(filename,'.mrl');
% fid = fopen(filename,'w');
% 
% num_st = size(JJ,2);
% num_p = 0;
% ba_joi = zeros(1,5);
% ba_joi = [0,46.1802765498601,-92.3605530997202,46.1802765498601,0];
% cha_joi = zeros(1,5);
% last_I_angle = 0;%上一步活动端的角度
% distance_to_wall_step_num_count = 0;%沿壁面方向量上抬的步数
% end_effector_index = [];  %1是1号吸盘，0是2号吸盘；这个变量待会帮我们判断要不要把关节角序列倒过来
% final_joint_angles = [];  %最终的关节角
% for i = 1:num_st
%     num_j = size(JJ{i},1);
%     if mod( i,2) ==1
%         if CREATE_G_CODE == 1
%             end_effector_index = [end_effector_index, 1]; %%记录是哪个吸盘在动       
%             last_I_angle = ba_joi(1, 5);        
%             distance_to_wall_step_num_count = 0;
%         end
%         for j = 1:num_j
%             if j == 1
%                 ba_joi = [ba_joi(1,1), JJ{i}(j,2:4), ba_joi(1,5)];
%             else
%                 cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
%                 ba_joi = ba_joi + cha_joi;            
%             end
%             
%             %%%%%%% 抬起后才将当前活动端清0
%             if CREATE_G_CODE == 1 
%                 distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
%                 if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
%                     ba_joi(1, 5) = 0;
%                 end
%             end
%             final_joint_angles = [final_joint_angles; [ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5)]];
% %             fprintf(fid,'P%d=',j+num_p);
% %             fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
% %             fprintf(fid,'\r\n');            
%         end
%     else
%         if CREATE_G_CODE == 1
%             end_effector_index = [end_effector_index, 0]; %%记录是哪个吸盘在动
%             last_I_angle = ba_joi(1, 1);
%             distance_to_wall_step_num_count = 0;
%         end
%         for j = 1:num_j
%             if j == 1
%                 ba_joi = [ba_joi(1,1), JJ{i}(j,4:-1:2), ba_joi(1,5)];
%             else
%                 cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
%                 ba_joi = ba_joi +[cha_joi(5),cha_joi(4),cha_joi(3),cha_joi(2),cha_joi(1)];       
%             end
%             
%             %%%%%%% 抬起后才将当前活动端清0
%             if CREATE_G_CODE == 1
%                 distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
%                 if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
%                     ba_joi(1, 1) = 0;
%                 end
%             end
%             final_joint_angles = [final_joint_angles; [ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5)]];
% %             fprintf(fid,'P%d=',j+num_p);
% %             fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
% %             fprintf(fid,'\r\n');            
%         end
%         
%     end
%     
%     %%%%%%%%%%%%%活动端的吸盘清0动作的分步
%     if CREATE_G_CODE == 1
%         delta_I_angle = last_I_angle / DISTANCE2WALL_STEP_NUM;
%         tmp_joint_angle = final_joint_angles(num_p + DISTANCE2WALL_STEP_NUM);
%         tmp_joint_angles = [];
%         for k = 1 : DISTANCE2WALL_STEP_NUM
%             if end_effector_index(size(end_effector_index, 2)) == 1
%                 tmp_joint_angle(1,5) = tmp_joint_angle(1,5) + delta_I_angle;
%             else
%                 tmp_joint_angle(1,1) = tmp_joint_angle(1,1) + delta_I_angle;
%             end
%         tmp_joint_angles = [tmp_joint_angles;tmp_joint_angle];
%         num_read_to_be_0 = num_p+DISTANCE2WALL_STEP_NUM;
%         %%%%%%%%%插入位置：活动端完全抬起但I关节未清0时
%         final_joint_angles = [final_joint_angles(1:num_read_to_be_0,:);...
%                                 tmp_joint_angles;...
%                                 final_joint_angles(num_read_to_be_0:num_read_to_be_0 +num_j-1,:)];
%         end
%     end
%     num_p = num_p +num_j-1 + DISTANCE2WALL_STEP_NUM;
%     step_inter(i) = num_p;
% end
% 
% for i = 1:num_p
%     if end_effector_index(i) == 1
%         fprintf(fid,'P%d=',j+num_p);
%         fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
%         fprintf(fid,'\r\n');   
%     else
%         fprintf(fid,'P%d=',j+num_p);
%         fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
%         fprintf(fid,'\r\n');   
%     end
% end
% 
% for i = 1:num_p
%      fprintf(fid,'MOVJ P%d, V70, Z1;',i);
%       fprintf(fid,'\r\n'); 
%       for j = 1:num_st-1
%           if i == step_inter(j)
%              fprintf(fid,'Delay(T5000);');
%              fprintf(fid,'\r\n'); 
%           end
%       end
% end
% fclose(fid);
% end

% function [  ] = jointangle_transform( JJ )
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2020/6/30备份
% %将关节角数据转换成攀爬机器人离线运动代码
% CREATE_G_CODE = 1;
% DISTANCE2WALL_STEP_NUM = 9;%沿壁面方向量上抬的步数
% filename = strcat('G_CODE_',strrep(datestr(now,26),'/','_'));
% filename = strcat(filename,'_');
% filename = strcat(filename,strrep(datestr(now,13),':','_'));
% filename = strcat(filename,'_');
% filename = strcat(filename,num2str(DISTANCE2WALL_STEP_NUM));
% filename = strcat(filename,'cm');
% filename = strcat(filename,'.mrl');
% fid = fopen(filename,'w');
% 
% num_st = size(JJ,2);
% num_p = 0;
% ba_joi = zeros(1,5);
% ba_joi = [0,46.1802765498601,-92.3605530997202,46.1802765498601,0];
% cha_joi = zeros(1,5);
% last_I_angle = 0;%上一步活动端的角度
% distance_to_wall_step_num_count = 0;%沿壁面方向量上抬的步数
% for i = 1:num_st
%     num_j = size(JJ{i},1);
%     if mod( i,2) ==1
%         last_I_angle = ba_joi(1, 5);
%         if CREATE_G_CODE == 1
%             distance_to_wall_step_num_count = 0;
%         end
%         for j = 1:num_j
%             if j == 1
%                 ba_joi = [ba_joi(1,1), JJ{i}(j,2:4), ba_joi(1,5)];
%             else
%                 cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
%                 ba_joi = ba_joi + cha_joi;            
%             end
%             
%             %%%%%%% 抬起后才将当前活动端清0
%             if CREATE_G_CODE == 1 
%                 distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
%                 if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
%                     ba_joi(1, 5) = 0;
%                 end
%             end
%             
%              fprintf(fid,'P%d=',j+num_p);
%               %fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',JJ{i}(j,1),JJ{i}(j,2),JJ{i}(j,3),JJ{i}(j,4),JJ{i}(j,5));
%               fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
%               fprintf(fid,'\r\n');            
%         end
%     else
%         last_I_angle = ba_joi(1, 1);
%         if CREATE_G_CODE == 1
%             distance_to_wall_step_num_count = 0;
%         end
%         for j = 1:num_j
%             if j == 1
%                 ba_joi = [ba_joi(1,1), JJ{i}(j,4:-1:2), ba_joi(1,5)];
%             else
%                 cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
%                 ba_joi = ba_joi +[cha_joi(5),cha_joi(4),cha_joi(3),cha_joi(2),cha_joi(1)];       
%             end
%             
%             %%%%%%% 抬起后才将当前活动端清0
%             if CREATE_G_CODE == 1
%                 distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
%                 if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
%                     ba_joi(1, 1) = 0;
%                 end
%             end
%             
%             fprintf(fid,'P%d=',j+num_p);
%              % fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',JJ{i}(j,5),JJ{i}(j,4),JJ{i}(j,3),JJ{i}(j,2),JJ{i}(j,1));
%               fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
%               fprintf(fid,'\r\n');            
%         end
%     end
%     
%     num_p = num_p +num_j-1;
%     step_inter(i) = num_p;
% end
% for i = 1:num_p
%      fprintf(fid,'MOVJ P%d, V70, Z1;',i);
%       fprintf(fid,'\r\n'); 
%       for j = 1:num_st-1
%       if i == step_inter(j)
% %           fprintf(fid,'\r\n');
%         fprintf(fid,'Delay(T5000);');
%         fprintf(fid,'\r\n'); 
%       end
%       end
% end
% fclose(fid);
% end

% function [  ] = jointangle_transform( JJ )
% %将关节角数据转换成攀爬机器人离线运动代码
% CREATE_G_CODE = 1;
% DISTANCE2WALL_STEP_NUM = 9;%沿壁面方向量上抬的步数
% filename = strcat('G_CODE_',strrep(datestr(now,26),'/','_'));
% filename = strcat(filename,'_');
% filename = strcat(filename,strrep(datestr(now,13),':','_'));
% filename = strcat(filename,'_');
% filename = strcat(filename,num2str(DISTANCE2WALL_STEP_NUM));
% filename = strcat(filename,'cm');
% filename = strcat(filename,'.mrl');
% fid = fopen(filename,'w');
% 
% num_st = size(JJ,2);
% num_p = 0;
% ba_joi = zeros(1,5);
% ba_joi = [0,46.1802765498601,-92.3605530997202,46.1802765498601,0];
% cha_joi = zeros(1,5);
% last_I_angle = 0;%上一步活动端的角度
% joint_angles_num = 0;
% distance_to_wall_step_num_count = 0;%沿壁面方向量上抬的步数
% end_effector_index = [];  %1是1号吸盘，0是2号吸盘；这个变量待会帮我们判断要不要把关节角序列倒过来
% final_joint_angles = [];  %最终的关节角
% for i = 1:num_st
%     num_j = size(JJ{i},1);
%     if mod( i,2) ==1
%         if CREATE_G_CODE == 1
%             end_effector_index = [end_effector_index, 1]; %%记录是哪个吸盘在动       
%             last_I_angle = ba_joi(1, 5);        
%             distance_to_wall_step_num_count = 0;
%         end
%         
%         joint_angles_num = floor(0.5 * num_j) - DISTANCE2WALL_STEP_NUM;
%         delta_angle = last_I_angle / joint_angles_num;
%         
%         for j = 1:num_j
%             if j == 1
%                 ba_joi = [ba_joi(1,1), JJ{i}(j,2:4), ba_joi(1,5)];
%             else
%                 cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
%                 ba_joi = ba_joi + cha_joi;            
%             end
%             
%             %%%%%%% 抬起后才将当前活动端清0
%             if CREATE_G_CODE == 1 
%                 distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
%                 if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
% %                     if distance_to_wall_step_num_count <= floor(0.5 * num_j)
% %                         ba_joi(1, 5) = ba_joi(1, 5) + delta_angle;
% %                     else
%                         ba_joi(1, 5) = 0;
% %                     end
%                 end
%             end
%             final_joint_angles = [final_joint_angles; [ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5)]];
% %             fprintf(fid,'P%d=',j+num_p);
% %             fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
% %             fprintf(fid,'\r\n');            
%         end
%     else
%         if CREATE_G_CODE == 1
%             end_effector_index = [end_effector_index, 0]; %%记录是哪个吸盘在动
%             last_I_angle = ba_joi(1, 1);
%             distance_to_wall_step_num_count = 0;
%         end
%         
%         joint_angles_num = floor(0.5 * num_j) - DISTANCE2WALL_STEP_NUM;
%         delta_angle = last_I_angle / joint_angles_num;
%         
%         for j = 1:num_j
%             if j == 1
%                 ba_joi = [ba_joi(1,1), JJ{i}(j,4:-1:2), ba_joi(1,5)];
%             else
%                 cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
%                 ba_joi = ba_joi +[cha_joi(5),cha_joi(4),cha_joi(3),cha_joi(2),cha_joi(1)];       
%             end
% 
%             %%%%%%% 抬起后才将当前活动端清0
%             if CREATE_G_CODE == 1
%                 distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
%                 if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
% %                     if distance_to_wall_step_num_count <= floor(0.5 * num_j)
% %                         ba_joi(1, 1) = ba_joi(1, 1) + delta_angle;
% %                     else
%                         ba_joi(1, 1) = 0;
% %                     end
%                 end
%             end
%             
%             final_joint_angles = [final_joint_angles; [ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5)]];
% %             fprintf(fid,'P%d=',j+num_p);
% %             fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
% %             fprintf(fid,'\r\n');            
%         end
%         
%     end
%     
%     %%%%%%%%%%%%%活动端的吸盘清0动作的分步
%     if CREATE_G_CODE == 1
%         delta_I_angle = last_I_angle / DISTANCE2WALL_STEP_NUM;
%         tmp_joint_angle = final_joint_angles(num_p + DISTANCE2WALL_STEP_NUM,:);
%         tmp_joint_angles = [];
%         for k = 1 : DISTANCE2WALL_STEP_NUM
%             if end_effector_index(size(end_effector_index, 2)) == 1
%                 tmp_joint_angle(1,5) = tmp_joint_angle(1,5) + delta_I_angle;
%             else
%                 tmp_joint_angle(1,1) = tmp_joint_angle(1,1) + delta_I_angle;
%             end
%         tmp_joint_angles = [tmp_joint_angles;tmp_joint_angle];
%         num_read_to_be_0 = num_p+DISTANCE2WALL_STEP_NUM;
%         %%%%%%%%%插入位置：活动端完全抬起但I关节未清0时
%         final_joint_angles = [final_joint_angles(1:num_read_to_be_0,:);...
%                                 tmp_joint_angles;...
%                                 final_joint_angles(num_read_to_be_0+1:num_j,:)];
%         end
%     end
%     num_p = num_p +num_j-1 + DISTANCE2WALL_STEP_NUM;
%     step_inter(i) = num_p;
% end
% 
% for i = 1:num_p
%     if end_effector_index(i) == 1
%         fprintf(fid,'P%d=',j+num_p);
%         fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
%         fprintf(fid,'\r\n');   
%     else
%         fprintf(fid,'P%d=',j+num_p);
%         fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
%         fprintf(fid,'\r\n');   
%     end
% end
% 
% for i = 1:num_p
%      fprintf(fid,'MOVJ P%d, V70, Z1;',i);
%       fprintf(fid,'\r\n'); 
%       for j = 1:num_st-1
%           if i == step_inter(j)
%              fprintf(fid,'Delay(T5000);');
%              fprintf(fid,'\r\n'); 
%           end
%       end
% end
% fclose(fid);
% end

function [  ] = jointangle_transform( JJ )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2020/6/30备份
%将关节角数据转换成攀爬机器人离线运动代码
CREATE_G_CODE = 1;
DISTANCE2WALL_STEP_NUM = 15;%沿壁面方向量上抬的步数
filename = strcat('G_CODE_',strrep(datestr(now,26),'/','_'));
filename = strcat(filename,'_');
filename = strcat(filename,strrep(datestr(now,13),':','_'));
filename = strcat(filename,'_');
filename = strcat(filename,num2str(DISTANCE2WALL_STEP_NUM));
filename = strcat(filename,'cm');
filename = strcat(filename,'.mrl');
fid = fopen(filename,'w');

num_st = size(JJ,2);
num_p = 0;
ba_joi = zeros(1,5);
ba_joi = [69.8421099139444,25.3224921700036,-50.6465908029406,25.3240986329370,69.8421099139444];
cha_joi = zeros(1,5);
last_I_angle = 0;%上一步活动端的角度
distance_to_wall_step_num_count = 0;%沿壁面方向量上抬的步数
for i = 1:num_st
    num_j = size(JJ{i},1);
    if mod( i,2) ==1
        last_I_angle = ba_joi(1, 5);
        if CREATE_G_CODE == 1
            distance_to_wall_step_num_count = 0;
        end
        
         joint_angles_num = floor(0.5 * num_j) - DISTANCE2WALL_STEP_NUM;
         delta_angle = last_I_angle / joint_angles_num;
        
        for j = 1:num_j
            if j == 1
                ba_joi = [ba_joi(1,1), JJ{i}(j,2:4), ba_joi(1,5)];
            else
                cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
                ba_joi = ba_joi + cha_joi;            
            end
            
            %%%%%%% 抬起后才将当前活动端清0
            if CREATE_G_CODE == 1 
                distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
                if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
                    if distance_to_wall_step_num_count <= floor(0.5 * num_j)
                        ba_joi(1, 5) = ba_joi(1, 5) - delta_angle;
                    else
                        ba_joi(1, 5) = 0;
                    end
                end
            end
            
             fprintf(fid,'P%d=',j+num_p);
              %fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',JJ{i}(j,1),JJ{i}(j,2),JJ{i}(j,3),JJ{i}(j,4),JJ{i}(j,5));
              fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
              fprintf(fid,'\r\n');            
        end
    else
        last_I_angle = ba_joi(1, 1);
        if CREATE_G_CODE == 1
            distance_to_wall_step_num_count = 0;
        end
        
         joint_angles_num = floor(0.5 * num_j) - DISTANCE2WALL_STEP_NUM;
         delta_angle = last_I_angle / joint_angles_num;
        
        for j = 1:num_j
            if j == 1
                ba_joi = [ba_joi(1,1), JJ{i}(j,4:-1:2), ba_joi(1,5)];
            else
                cha_joi = JJ{i}(j,:)-JJ{i}(j-1,:);
                ba_joi = ba_joi +[cha_joi(5),cha_joi(4),cha_joi(3),cha_joi(2),cha_joi(1)];       
            end
            
            %%%%%%% 抬起后才将当前活动端清0
            if CREATE_G_CODE == 1
                distance_to_wall_step_num_count = distance_to_wall_step_num_count + 1;
                if distance_to_wall_step_num_count >= DISTANCE2WALL_STEP_NUM
                    if distance_to_wall_step_num_count <= floor(0.5 * num_j)
                        ba_joi(1, 1) = ba_joi(1, 1) - delta_angle;
                    else
                        ba_joi(1, 1) = 0;
                    end
                end
            end
            
            fprintf(fid,'P%d=',j+num_p);
             % fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',JJ{i}(j,5),JJ{i}(j,4),JJ{i}(j,3),JJ{i}(j,2),JJ{i}(j,1));
              fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
              fprintf(fid,'\r\n');            
        end
    end
    
    num_p = num_p +num_j-1;
    step_inter(i) = num_p;
end
for i = 1:num_p
     fprintf(fid,'MOVJ P%d, V70, Z1;',i);
      fprintf(fid,'\r\n'); 
      for j = 1:num_st-1
      if i == step_inter(j)
%           fprintf(fid,'\r\n');
        fprintf(fid,'Delay(T5000);');
        fprintf(fid,'\r\n'); 
      end
      end
end
fclose(fid);
end
