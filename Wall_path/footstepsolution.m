function [ se_footstep3d,numst ] = footstepsolution( ps,pe,map_border,obs_border,obs_height)
%��һ�������ɢ������滮���


     [pathxy] = astar_path(0.05,ps,pe,map_border,obs_border,obs_height);%A*�㷨���������Ǳ���滮���м�㣬������ʼ��
    [ pathstr,dis] = path_des( pathxy,ps,pe,map_border,obs_border,obs_height);%������һ��  path= [ps;pathxy1;pe];  ����se_footstep3d�ǰ�����ʼ���
    [ se_footstep3d ] = footstep_sequence( pathstr,map_border,obs_border,obs_height );
    numst = size(se_footstep3d,1)-1;% ����
    
end

