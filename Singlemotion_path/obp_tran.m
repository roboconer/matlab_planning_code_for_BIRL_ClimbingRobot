function[obs_sur] =obp_tran(obp)%将代表障碍物的8个三维空间点转化为障碍物元素（5个面元素和8个线段元素）
num_ob = size(obp,1)/8;
obs_sur = zeros(5,4*3,num_ob);
obs_line = zeros(8,2*3,num_ob);
for i = 1:num_ob
    for j=1:4
        if j==4
           obs_sur(j,:,i)=[obp(8*i-4,:),obp(8*i-7,:),obp(8*i-3,:),obp(8*i,:)]; 
        else
           obs_sur(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:),obp(8*i-3+j,:),obp(8*i-4+j,:)];
        end  
    end
    obs_sur(5,:,i)=[obp(8*i-3,:),obp(8*i-2,:),obp(8*i-1,:),obp(8*i,:)];
end
for i=1:num_ob
    for j=1:4
        obs_line(j,:,i)=[obp(8*i-8+j,:),obp(8*i-4+j,:)];
    end
    for j=5:8
        if j==8
            obs_line(j,:,i)=[obp(8*i,:),obp(8*i-3,:)];
        else
            obs_line(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:)];
        end
    end
end
% for i=1:5
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%           plot3(sur_x*1000, sur_y*1000,sur_z*1000,'-g','LineWidth',2);
for i = 1: num_ob 
 fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
 patch('Faces',fac_obs,'Vertices',obp(i*8-7:i*8,:)*1000,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',1);
         hold on;
end
end

