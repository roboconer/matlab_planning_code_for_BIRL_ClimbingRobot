function [ obs_p ] = map_obs( map_border,obs_border,obs_height )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
% ���ϰ��������ת������������ϵ��()
%num_map = size(map_border,1);
    num_obs = floor(size(obs_border,1)/4); %�ϰ��ı�ʾ��ʽ���ϰ�����μӸ߶�
    Vz = cross(map_border(2,:)-map_border(1,:),map_border(3,:)-map_border(2,:));   %�������η�����
    Vz = Vz/norm(Vz);

    Vx = (map_border(2,:)-map_border(1,:))/norm(map_border(2,:)-map_border(1,:));   %���������ϵ����������ϵ�µı�ʾ
    Vy =  cross(Vz,Vx);

    stw_R = [Vx',Vy',Vz'];

    obs_p = [];
%���ϰ���ת��obs(pointx1,pointy1, pointx2, pointy2, height1...)�ĸ�ʽ
    for i = 1:num_obs
        obs_p(i*8-7:i*8-4,1:2) = obs_border(i*4-3:i*4,:);
        obs_p(i*8-3:i*8,1:2) = obs_border(i*4-3:i*4,:);
        obs_p(i*8-3:i*8,3) = obs_height(i);
    end
%���ϰ��������ת������������ϵ��
    for i = 1:num_obs
        for j = 1:4
            obs_p(i*8-8+j,:) = (stw_R*obs_p(i*8-8+j,:)'+ map_border(1,:)')' ;
            obs_p(i*8-4+j,:) = obs_p(i*8-8+j,:) + Vz*obs_height(i);
        end
    end

end

