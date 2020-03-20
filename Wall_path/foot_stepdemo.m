function [ se_footstep3d,numst ] = foot_stepdemo(  )
%壁面离散吸附点规划算例运行与算例分析
%   此处显示详细说明
    input_map;
    [ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height );               %导入地图并生成障碍区域
    num_bor = size(map_bor,1);
    Vn=[0 0 1];
    max_borx = max(map_bor(:,1));      
    max_bory = max(map_bor(:,2));
    min_borx = min(map_bor(:,1));
    min_bory = min(map_bor(:,2));
   ranx = max_borx-min_borx;
   rany = max_bory-min_bory;
    num_byp=size(byp_obs,1);  
    byp1_obs=zeros(num_byp,2); %绕开的障碍物区域
    num_acr=size(acr_obs,1);
    acr1_obs=zeros(num_acr,2); %跨越的障碍物区域
    num_att=size(att_obs,1);
    att1_obs=zeros(num_acr,2); %吸附的障碍物扩展区域
    k_byp=num_byp/4;
    k_acr=num_acr/4;
    k_att=num_att/4;
    byp1_obs=byp_obs(:,3:4);
    att1_obs=att_obs(:,3:4);
    acr1_obs=acr_obs(:,3:4);
    acr_stm1=zeros(k_acr,1);
    att_stm1=zeros(k_att,1);
    byp_stm1=zeros(k_byp,1);
         ps=[5.5   4.2 ];
         pe=[5.42   5.2];        
           ps=[ 3.3    0.5];
       pe=[4.4    6.5 ];
%             ps=[ 0.4    2.1];
% % %            ps=[ 3.2    6.3];
%         pe=[7.8    1.4 ];
     state_ps = 0;
    state_pe = 0;
    while state_ps == 0||state_pe == 0
           ps=[rand*ranx,rand*rany];
           pe=[rand*ranx,rand*rany];
        in_borps = insidepolygon(map_bor,ps);
        in_borpe = insidepolygon(map_bor,pe);
     for j=1:k_byp                                
        byp_stm1(j) = insidepolygon(byp1_obs(j*4-3:j*4,:),ps);
        if byp_stm1(j)==1 
             break;
        end
         byp_stm1(j) = insidepolygon(byp1_obs(j*4-3:j*4,:),pe);
        if byp_stm1(j)==1 
             break;
        end
      end
   for j=1:k_acr
            acr_stm1(j) = insidepolygon(acr1_obs(j*4-3:j*4,:),ps);
                 if acr_stm1(j)==1                 
                   break;
                 end
           acr_stm1(j) = insidepolygon(acr1_obs(j*4-3:j*4,:),pe);
                 if acr_stm1(j)==1                 
                   break;
                 end
    end
       for j=1:k_att             
                 att_stm1(j) = insidepolygon(att1_obs(j*4-3:j*4,:),ps);
                 if att_stm1(j)==1
                     break;
                 end
                att_stm1(j) = insidepolygon(att1_obs(j*4-3:j*4,:),pe);
                 if att_stm1(j)==1
                     break;
                 end
       end
      if in_borps==0||in_borpe==0||any(byp_stm1==1)||any(acr_stm1==1)||any(att_stm1)==1
          state_ps=0;
         state_pe = 0;
      else
          state_ps=1;
          state_pe=1;
          break;
      end  
    end
    [pathxy] = astar_path(0.05,ps,pe,map_border,obs_border,obs_height);
    [ pathstr,dis] = path_des( pathxy,ps,pe,map_border,obs_border,obs_height);
     [ se_footstep3d ] = footstep_sequence( pathstr,map_border,obs_border,obs_height );
     numst = size(se_footstep3d,1)-1;
     %plot_environment(se_footstep3d,map_border,obs_border,obs_height,Vn)
end

