function [ Ba,Ba_R,joint_ang] = motionplan2(x,st_op)
    %单步运动规划求解
    global link_1 link_2;
    link_1 = 0.329; %class 
    link_2 = 0.2932; %class
    [map_border, obstruct_borders, obstruct_height] = input_map2;
    wall_planning_points = x; %class
    step_type = st_op; %class
    valiable_obstruct_index = 0;%class
    [surface,num_point_each_surface,num_surfaces,Pstart,Pend] = surfaces;%class
    surface_step=cell(1,3);%class
    r = 0.1998;%class

    %对每个壁面做缩放
    for i=1:num_surfaces
        v1= (surface{i}(num_point_each_surface(i),:)-surface{i}(1,:))/norm(surface{i}(num_point_each_surface(i),:)-surface{i}(1,:));
        v2= (surface{i}(2,:)-surface{i}(1,:))/norm(surface{i}(2,:)-surface{i}(1,:));
        ang_cos = dot(v1,v2);
        ang_sin = sqrt(1-ang_cos^2);
        l = r/ang_sin;
        shrink_surface{i}(1,:) = surface{i}(1,:)+ v1*l +v2*l;

        v1=( surface{i}(1,:)-surface{i}(num_point_each_surface(i),:))/norm(surface{i}(1,:)-surface{i}(num_point_each_surface(i),:));
        v2= (surface{i}(num_point_each_surface(i)-1,:)-surface{i}(num_point_each_surface(i),:))/norm(surface{i}(num_point_each_surface(i)-1,:)-surface{i}(num_point_each_surface(i),:));
        ang_cos = dot(v1,v2);
        ang_sin = sqrt(1-ang_cos^2);
        l = r/ang_sin;
        shrink_surface{i}(num_point_each_surface(i),:) = surface{i}(num_point_each_surface(i),:)+ v1*l +v2*l;

        for j=2:num_point_each_surface(i)-1
            v1= (surface{i}(j+1,:)-surface{i}(j,:))/norm(surface{i}(j+1,:)-surface{i}(j,:));
            v2= (surface{i}(j-1,:)-surface{i}(j,:))/norm(surface{i}(j-1,:)-surface{i}(j,:));
            ang_cos = dot(v1,v2);
            ang_sin = sqrt(1-ang_cos^2);
            l = r/ang_sin;
            shrink_surface{i}(j,:) = surface{i}(j,:)+ v1*l +v2*l;
        end
    end
    surface_step_index =zeros(1,50);

    %% 三个点构成一个动作，记录下这三个点所在平面
    for j = 1:size(wall_planning_points,1) - 2
        start_point = wall_planning_points(j,:);%start point
        base_point = wall_planning_points(j+1,:);%base point
        end_point = wall_planning_points(j+2,:);%end point
        for i= 1:num_surfaces
            if insidepoly3(base_point,surface{i}) ==1
                surface_step{1} = surface{i};
                surface_step_index(1) = i;
            end
            if insidepoly3(end_point,surface{i}) ==1
                surface_step{2} = surface{i};
                surface_step_index(2) = i;
            end
            if insidepoly3(start_point,surface{i}) ==1
                surface_step{3} = surface{i};
                surface_step_index(3) = i;
            end
        end

        step_option = step_type(j);
        base_surface = surface_step{1};

        %% 计算基座坐标系
        Vnm=cross(base_surface(2,:)-base_surface(1,:),base_surface(3,:)-base_surface(2,:));   %计算多边形法向量
        Vnm=Vnm/norm(Vnm);
        bas_x = (base_surface(2,:)-base_surface(1,:))/norm(base_surface(2,:)-base_surface(1,:));   %求机器人运动的基坐标系
        bas_z = Vnm;
        bas_y = cross(bas_z,bas_x);
        bas_R_w_b = [bas_x',bas_y',bas_z'];%R_w_b

        %% 将障碍物坐标点转换到壁面坐标系下（这个数组obs_p存放了全部壁面障碍点）
        num_obstructs = 1;
        for i = 1:num_surfaces%求解机器人工作空间内的有效障碍物
            if obstruct_height{i} == 0
                continue;
            end
            num_ob2 = length(obstruct_height{i});
            obs_p(num_obstructs*8-7:(num_obstructs + num_ob2 - 1)*8,:)  = map_obs( shrink_surface{i},obstruct_borders{i},obstruct_height{i} );
            % obs_p(1:8,:) = map_obs( shrink_surface{i},obstruct_borders{i},obstruct_height{i} );
            num_obstructs = num_obstructs + num_ob2;
        end

        %% 判断机器人工作空间内是否有障碍物
        testobj =base_point + Vnm*link_1;
        obs_p= [];
        num_obstructs = 1;
        valiable_obstruct_index = [] ;
        if size(obs_p,1)>0
            for i=1:floor(size(obs_p,1)/8)
                [ dis,po1,po2] = dis_compute(testobj,obs_p(i*8-7:i*8,:),1);
                if dis <= 2*link_2+link_1 %伸直状态
                    valiable_obstruct_index(num_obstructs) = i;
                    % obs_p2(num_obstructs*8+1:num_obstructs*8+8,:) = obs_p(i*8-7:i*8,:);
                    num_obstructs = num_obstructs +1;
                end
            end
        end
        if size(valiable_obstruct_index,1) ==0
            obs_p2 = zeros(4,3);
        else
            obs_p2 = zeros(length(valiable_obstruct_index)*8,3);
            for i = 1:length(valiable_obstruct_index)
                obs_p2(i*8-7:i*8,:) = obs_p(valiable_obstruct_index(i)*8-7:valiable_obstruct_index(i)*8,:);
            end
        end

        %% 翻转末端Z轴
        if step_option ==1
            stp_R_w_s = eye(3);
            stp_R_w_s(1:3,2) = stp_R_w_s(1:3,2)*-1;
            stp_R_w_s(1:3,3) = stp_R_w_s(1:3,3)*-1;
        else
            %求机器人运动的起点坐标系
            %先求出起始壁面的姿态，转换到基座坐标系下，然后将y、z轴方向取反
            sta_x = (surface_step{3}(2,:)-surface_step{3}(1,:))/norm(surface_step{3}(2,:)-surface_step{3}(1,:));
            Vnm3=cross(surface_step{3}(2,:)-surface_step{3}(1,:),surface_step{3}(3,:)-surface_step{3}(2,:));   %计算多边形法向量
            Vnm3=Vnm3/norm(Vnm3);
            sta_z = Vnm3;
            sta_y = cross(sta_z,sta_x);
            stp_R2 =[sta_x',sta_y',sta_z'];
            stp_R_w_s =  bas_R_w_b'*stp_R2;%R_b_s
            stp_R_w_s(1:3,2) = stp_R_w_s(1:3,2)*-1;
            stp_R_w_s(1:3,3) = stp_R_w_s(1:3,3)*-1;
        end

        first_joint = [0  20  -40  20 0];%@@@@@@@@@@@@@@为什么一开始要指定这个角度
        % [-23.0048561393523,-27.1477662270062,129.071883883613]
        %这个是上一次的基座、起始壁面姿态

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 基座交替变化
        if j > 1   %
            bas_R_w_b = last_bas_R;
            stp_R_w_s = last_stp_R;
            first_joint = last_joint;
        end
        %% 单步规划

        Ba(j,:)=base_point;
        Ba_R{j}=bas_R_w_b;

        if step_option ==1    %% 壁面内单步运动规划
            disp(j)
            tic
            [last_bas_R,last_stp_R,last_joint,joint_ang{j}] = colli_avoidance( base_surface,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
            toc
        else                  %% 壁面过渡运动规划
            num_obsur = 3;
            if any(surface_step_index(1:3) == 0)
                h=msgbox('没有求解到落足点对应的壁面','warn');
                uiwait(h,2);
                return;
            end

            %判断工作空间内有可能碰到的壁面
            for i = 1:num_surfaces
                [ dis,po1,po2] = dis_compute(testobj,surface{i},3);
                if dis <= 2*link_2+link_1
                    if any(surface_step_index(1:3)==i)
                        continue;
                    else
                        surface_step_index(num_obsur +1) = i;
                        surface_step{num_obsur+1} = surface{i};
                        num_obsur = num_obsur +1;
                    end
                end
            end
            tic
            disp(j)
            [last_bas_R,last_stp_R,last_joint,joint_ang{j}] = colli_avoidance2( surface_step,start_point,end_point,base_point,obs_p2,bas_R_w_b,stp_R_w_s,first_joint,1);
            toc
        end
    end
