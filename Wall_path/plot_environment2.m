function [] =plot_environment2(se_footstep,map_border,obs_border,obs_height,Vn)
%水平面障碍物与机器人立体绘图
[map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height); 
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
num_bor=size(map_bor,1);
num_map=size(map_bor,1);
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
map1_bor=map_bor*1000;
byp1_obs=byp_obs*1000;
acr1_obs=acr_obs*1000;
att1_obs=att_obs*1000;
%clf;
for i=1:num_bor
fac(1,i) =i;
end
figure(2)
clf
patch('Faces',fac,'Vertices',pat_bor,'FaceVertexCData',[0.4 0.4 0.4],'FaceColor','flat');
 hold on
 
  plot(map1_bor(:,1),map1_bor(:,2),'-r','LineWidth',2);
   
    ppk=[map1_bor(num_map,:);map1_bor(1,:)];
    plot(ppk(:,1),ppk(:,2),'-r','LineWidth',2);
for i=1:k_byp
    plot(byp1_obs(4*i-3:4*i,1),byp1_obs(4*i-3:4*i,2),'-b','LineWidth',2);
    ppk=[byp1_obs(4*i,1:2);byp1_obs(4*i-3,1:2)];
    plot(ppk(:,1),ppk(:,2),'-b','LineWidth',2);
  
end

for i=1:k_acr
    plot(acr1_obs(4*i-3:4*i,1),acr1_obs(4*i-3:4*i,2),'-g','LineWidth',2);
    ppk=[acr1_obs(4*i,1:2);acr1_obs(4*i-3,1:2)];
    plot(ppk(:,1),ppk(:,2),'-g','LineWidth',2);
   
end

for i=1:k_att
    plot(att1_obs(4*i-3:4*i,1),att1_obs(4*i-3:4*i,2),'-y','LineWidth',2);
    ppk=[att1_obs(4*i,1:2);att1_obs(4*i-3,1:2)];
    plot(ppk(:,1),ppk(:,2),'-y','LineWidth',2);
   
end
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
% for i=1:k_acr
% pat_acr(8*i-7:8*i-4,:)=[acr_obs(4*i-3:4*i,1),acr_obs(4*i-3:4*i,2),ones(4,1)*hei_obs(k_byp+i)]*1000;
% pat_acr(8*i-3:8*i,:)=[acr_obs(4*i-3:4*i,1),acr_obs(4*i-3:4*i,2),zeros(4,1)]*1000;
% end
% for i=1:k_acr
%     fac_acr(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
% end
% if k_acr>0
% patch('Faces',fac_acr,'Vertices',pat_acr,'FaceVertexCData',[0.5 0.98 0],'FaceColor','flat');
% end
% for i=1:k_att
% pat_att(8*i-7:8*i-4,:)=[att_obs(4*i-3:4*i,1),att_obs(4*i-3:4*i,2),ones(4,1)*hei_obs(k_byp+k_acr+i)]*1000;
% pat_att(8*i-3:8*i,:)=[att_obs(4*i-3:4*i,1),att_obs(4*i-3:4*i,2),zeros(4,1)]*1000;
% end
% for i=1:k_att
%     fac_att(i*6-5:i*6,:) = [8*(i-1)+1,8*(i-1)+2,8*(i-1)+3,8*(i-1)+4;8*(i-1)+5,8*(i-1)+6,8*(i-1)+7,8*(i-1)+8;8*(i-1)+1,8*(i-1)+2,8*(i-1)+6,8*(i-1)+5;8*(i-1)+2,8*(i-1)+3,8*(i-1)+7,8*(i-1)+6;8*(i-1)+3,8*(i-1)+4,8*(i-1)+8,8*(i-1)+7;8*(i-1)+4,8*(i-1)+1,8*(i-1)+5,8*(i-1)+8];
% end
% if k_att>0
% patch('Faces',fac_att,'Vertices',pat_att,'FaceVertexCData',[1 1 0],'FaceColor','flat');
% end
axis equal 

axis([0,8000,0,7000,0,2000]);
 set(gca,'ZTick',1000:1000:2000);
 set(gca,'YTick',1000:1000:7000);
 set(gca,'XTick',0:1000:8000);
set(gca,'xticklabel',mod(0:1:8,9));
set(gca,'yticklabel',mod(1:1:7,8));
fz =17;

 set(gca,'zticklabel',mod(1:1:2,3));
hxl= xlabel('\it{X}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');        %设置坐标轴格式
%set(hxl,'Position',[1000 -1500 0]);
%set(hxl,'Position',[500 -1100 0]);

hyl= ylabel('\it{Y}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');
hzl= zlabel('\it{Z}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');
%set(hyl,'Position',[11200 0 0]);
%set(hyl,'Position',[-1000 500 0]);
set(gca,'Fontname','Times New Roman');
set(gca,'fontsize',fz);
se_footstep3d = se_footstep;
%DrawRobotfoot(se_footstep3d,Vn);
end