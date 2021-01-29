function [ obs_p ] = map_obs( map_border,obs_border,obs_height )
%UNTITLED 此处显示有关此函数的摘要
% 将障碍物坐标点转换到世界坐标系下()
%num_map = size(map_border,1);
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
%将障碍物坐标点转换到世界坐标系下
    for i = 1:num_obs
        for j = 1:4
            obs_p(i*8-8+j,:) = (stw_R*obs_p(i*8-8+j,:)'+ map_border(1,:)')' ;
            obs_p(i*8-4+j,:) = obs_p(i*8-8+j,:) + Vz*obs_height(i);
        end
    end

end

