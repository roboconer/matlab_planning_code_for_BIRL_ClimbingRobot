function [] =plot_environment(se_footstep)
%ÕÏ°­ÎïÁ¢Ìå»æÍ¼
[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map; 
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
num_bor=size(map_bor,1);
k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;
fac_byp=zeros(6*k_byp,4);
pat_byp = zeros(2*num_byp,3);
fac_acr=zeros(6*k_acr,4);
pat_acr = zeros(2*num_acr,3);
fac_att=zeros(6*k_att,4);
pat_att = zeros(2*num_att,3);
pat_bor=[map_bor(:,1),map_bor(:,2),zeros(num_bor,1)]*1000;
for i=1:num_bor
fac(1,i) =i;
end
patch('Faces',fac,'Vertices',pat_bor,'FaceVertexCData',[0.4 0.4 0.4],'FaceColor','flat');
for i=1:k_byp
pat_byp(8*i-7:8*i-4,:)=[byp_obs(4*i-3:4*i,1),byp_obs(4*i-3:4*i,2),ones(4,1)*hei_obs(i)]*1000;
pat_byp(8*i-3:8*i,:)=[byp_obs(4*i-3:4*i,1),byp_obs(4*i-3:4*i,2),zeros(4,1)]*1000;
end
for i=1:k_byp
    fac_byp(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
end
if k_byp>0
patch('Faces',fac_byp,'Vertices',pat_byp,'FaceVertexCData',[0.3 0.45 1],'FaceColor','flat');
end
for i=1:k_acr
pat_acr(8*i-7:8*i-4,:)=[acr_obs(4*i-3:4*i,1),acr_obs(4*i-3:4*i,2),ones(4,1)*hei_obs(k_byp+i)]*1000;
pat_acr(8*i-3:8*i,:)=[acr_obs(4*i-3:4*i,1),acr_obs(4*i-3:4*i,2),zeros(4,1)]*1000;
end
for i=1:k_acr
    fac_acr(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
end
if k_acr>0
patch('Faces',fac_acr,'Vertices',pat_acr,'FaceVertexCData',[0.5 0.98 0],'FaceColor','flat');
end
for i=1:k_att
pat_att(8*i-7:8*i-4,:)=[att_obs(4*i-3:4*i,1),att_obs(4*i-3:4*i,2),ones(4,1)*hei_obs(k_byp+k_acr+i)]*1000;
pat_att(8*i-3:8*i,:)=[att_obs(4*i-3:4*i,1),att_obs(4*i-3:4*i,2),zeros(4,1)]*1000;
end
for i=1:k_att
    fac_att(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
end
if k_acr>0
patch('Faces',fac_att,'Vertices',pat_att,'FaceVertexCData',[1 1 0],'FaceColor','flat');
end
se_footstep3d = se_footstep;
DrawRobotDemo;
end