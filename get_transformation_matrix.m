function [Transform, pose,end_effector_pose_1, end_effector_pose_2]  = get_transformation_matrix( joint_angles, wallFootStep_Cell )
%   get_transformation_matrix Summary of this function goes here
%   get_transformation_matrix:输入关节角序列，获得对应转换矩阵（T_w_r）序列
%   wallFootStep_Cell:见main函数
%   Transform: 1xN cell,转换矩阵的元胞数组
%   joint_angles: 1xN cell,各步的关节角序列
%   pose:nx3 矩阵，两端的运动轨迹(motion capture追踪环的位置)
%   x, y, z:pose的第1、2、3列
SHOW = 1;

[surface,num_point_each_surface,num_surfaces,Pstart,Pend] = surfaces;
num_steps = size(joint_angles, 2);
num_frame = 0;  %每一步中有多少帧
end1_is_base = 1; %吸盘一是否为基座
joint_base = [0, 0, 0, 0, 0];  %单步关节角基准
joint_current = [0, 0, 0, 0, 0];  %当前关节角
if SHOW
    figure(1)
    plot_surface;
    hold on;
end

%计算基座坐标系
Vnm=cross(surface{1}(2,:)-surface{1}(1,:),surface{1}(3,:)-surface{1}(2,:));   %计算多边形法向量
Vnm=Vnm/norm(Vnm);
bas_x = (surface{1}(2,:)-surface{1}(1,:))/norm(surface{1}(2,:)-surface{1}(1,:));   %求机器人运动的基坐标系
bas_z = Vnm; 
bas_y = cross(bas_z,bas_x);

T_base_end = [ 1,  0,  0,  0;
      0,  1,  0,  0;
      0,  0,  1,  0;
      0,  0,  0,  1];
  
T_world_end = [ 1,  0,  0,  0;
      0,  1,  0,  0;
      0,  0,  1,  0;
      0,  0,  0,  1];                 
                  
T_world_base = [ 1,  0,  0,  0;
                0,  1,  0,  0;
                0,  0,  1,  0;
                0,  0,  0,  1];            
            
T_world_base(1:3,1:3) = [bas_x',bas_y',bas_z'];%R_w_b
T_world_base(1:3,4) = wallFootStep_Cell{1}(2,:)*1000;

angle_cell = {};
Transform = {};
pose = [];
end_effector_pose_1 = [];
end_effector_pose_2 = [];
% joint_current = joint_angles{1, 1}(1,:);

%%解初始角度
joint_current = joint_angles{1, 1}(1,:);

for step_index = 1 : num_steps  
    num_frame = size(joint_angles{1, step_index},1);
    
    % 单步
    for frame_index = 1 : num_frame - 1 
        
        joint_defference = joint_angles{1, step_index}(frame_index+1,:) - joint_angles{1, step_index}(frame_index,:);
       
        if end1_is_base  %吸盘1为基座           
            joint_current = joint_current + joint_defference;
            T_base_end = Kine5D(joint_current, 4);%这里的end可能是end1或end2，end1_is_base
            T_end_TrackingRing = [  1,  0,  0,  0;
                  0,  1,  0,  0;
                  0,  0,  1,  -137;%-137
                  0,  0,  0,  1]; 
        else  %吸盘2为基座        
            joint_current = joint_current + fliplr(joint_defference);
            T_base_end = Kine5D(joint_current, 4);%这里的end可能是end1或end2，end1_is_base
            T_base_end = inv(T_base_end); 
            T_end_TrackingRing = [  1,  0,  0,  0;
                      0,  1,  0,  0;
                      0,  0,  1,  137;%137
                      0,  0,  0,  1]; 
        end                
        T_world_end = T_world_base * T_base_end
        Transform{size(Transform, 1) + 1, 1} = T_world_end;
        T_world_TrackingRing = T_world_end * T_end_TrackingRing;  %追踪环的齐次矩阵
        pose(size(pose, 1)+1,:) = T_world_TrackingRing(1:3, 4)';
%         pose(size(pose, 1)+1,:) = T_world_end(1:3, 4)';
        if mod( step_index, 2 ) == 1
            end_effector_pose_1 = [end_effector_pose_1;T_world_TrackingRing(1:3, 4)'];     
        else
            end_effector_pose_2 = [end_effector_pose_2;T_world_TrackingRing(1:3, 4)'];
        end
        if SHOW
            plot3(T_world_TrackingRing(1, 4),T_world_TrackingRing(2, 4),T_world_TrackingRing(3, 4),'b*')
        end
    end
    end1_is_base = ~end1_is_base;
    T_world_base = T_world_end;
end

%%%%%%%%%%%将追踪环的位置写到文件中
csvwrite('end_effector_pose_1.csv', end_effector_pose_1);
csvwrite('end_effector_pose_2.csv', end_effector_pose_2);
end

