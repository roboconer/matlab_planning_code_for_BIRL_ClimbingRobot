function [ se_footstep3d,numst ] = footstepsolution( ps,pe,map_border,obs_border,obs_height)
%单一壁面的离散吸附点规划求解


     [pathxy] = astar_path(0.05,ps,pe,map_border,obs_border,obs_height);%A*算法，出来的是壁面规划的中间点，不含起始点
    [ pathstr,dis] = path_des( pathxy,ps,pe,map_border,obs_border,obs_height);%里面有一句  path= [ps;pathxy1;pe];  所以se_footstep3d是包含起始点的
    [ se_footstep3d ] = footstep_sequence( pathstr,map_border,obs_border,obs_height );
    numst = size(se_footstep3d,1)-1;% 步数
    
end

