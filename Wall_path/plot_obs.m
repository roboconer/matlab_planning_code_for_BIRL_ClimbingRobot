function [  ] = plot_obs( map_border,obs_border,obs_height,transR,pinit )
%UNTITLED 多壁面上的障碍物绘制
%   此处显示详细说明
[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height); 
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
num_bor=size(map_bor,1);
k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;
fac_byp=zeros(6*k_byp,4);
pat_byp = zeros(2*num_byp,3);
byp_3d = zeros(num_byp,3);
fac_acr=zeros(6*k_acr,4);
pat_acr = zeros(2*num_acr,3);
fac_att=zeros(6*k_att,4);
pat_att = zeros(2*num_att,3);
for i=1:k_byp
for j=1:4
pat_byp(8*i-8+j,:) = ((transR*[byp_obs(4*i-4+j,1),byp_obs(4*i-4+j,2),hei_obs(i)]')'+pinit)*1000;
pat_byp(8*i-4+j,:) = ((transR*[byp_obs(4*i-4+j,1),byp_obs(4*i-4+j,2),0]')'+pinit)*1000;
end
end
for i=1:k_byp
    fac_byp(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
end
if k_byp>0
patch('Faces',fac_byp,'Vertices',pat_byp,'FaceVertexCData',[0.3 0.45 1],'FaceColor','flat');
end

for i=1:k_acr
for j=1:4
pat_acr(8*i-8+j,:) = ((transR*[acr_obs(4*i-4+j,1),acr_obs(4*i-4+j,2),hei_obs(k_byp+i)]')'+pinit)*1000;
pat_acr(8*i-4+j,:) = ((transR*[acr_obs(4*i-4+j,1),acr_obs(4*i-4+j,2),0]')'+pinit)*1000;
end
end
for i=1:k_acr
    fac_acr(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
end
if k_acr>0
patch('Faces',fac_acr,'Vertices',pat_acr,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat');
end

for i=1:k_att
for j=1:4
pat_att(8*i-8+j,:) = ((transR*[att_obs(4*i-4+j,1),att_obs(4*i-4+j,2),hei_obs(k_byp+k_acr+i)]')'+pinit)*1000;
pat_att(8*i-4+j,:) = ((transR*[att_obs(4*i-4+j,1),att_obs(4*i-4+j,2),0]')'+pinit)*1000;
end
end
for i=1:k_att
    fac_att(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
end
if k_att>0
patch('Faces',fac_att,'Vertices',pat_att,'FaceVertexCData',[1 1 0],'FaceColor','flat');
end
end

