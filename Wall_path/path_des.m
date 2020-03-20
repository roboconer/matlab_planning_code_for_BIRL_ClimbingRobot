function [ pathstr,path_dis] = path_des( pathxy1,ps,pe,map_border,obs_border,obs_height)
%地图与路径描绘
num_path = size(pathxy1,1);
[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map(map_border,obs_border,obs_height);               %导入地图并生成障碍区域
num_map=size(map_bor,1);
num_byp=size(byp_obs,1);
num_acr=size(acr_obs,1);
num_att=size(att_obs,1);
k_byp=num_byp/4;
k_acr=num_acr/4;
k_att=num_att/4;
pat_bor=[map_bor(:,1),map_bor(:,2),zeros(num_map,1)]*1000;
map1_bor=map_bor*1000;
byp1_obs=byp_obs*1000;
acr1_obs=acr_obs*1000;
att1_obs=att_obs*1000;
for i=1:num_map
fac(1,i) =i;
end
figure(2)
clf
%patch('Faces',fac,'Vertices',pat_bor,'FaceVertexCData',[0.4 0.4 0.4],'FaceColor','flat');
  hold on
    plot(map1_bor(:,1),map1_bor(:,2),'-r','LineWidth',2);
  
    ppk=[map1_bor(num_map,:);map1_bor(1,:)];
    plot(ppk(:,1),ppk(:,2),'-r','LineWidth',2);
for i=1:k_byp
    plot(byp1_obs(4*i-3:4*i,1),byp1_obs(4*i-3:4*i,2),'-b','LineWidth',2);
    ppk=[byp1_obs(4*i,1:2);byp1_obs(4*i-3,1:2)];
    plot(ppk(:,1),ppk(:,2),'-b','LineWidth',2);
    plot(byp1_obs(4*i-3:4*i,3),byp1_obs(4*i-3:4*i,4),'-b','LineWidth',2,'Linestyle','- -');
    ppk=[byp1_obs(4*i,3:4);byp1_obs(4*i-3,3:4)];
    plot(ppk(:,1),ppk(:,2),'-b','LineWidth',2,'Linestyle','- -');
end

for i=1:k_acr
    plot(acr1_obs(4*i-3:4*i,1),acr1_obs(4*i-3:4*i,2),'-g','LineWidth',2);
    ppk=[acr1_obs(4*i,1:2);acr1_obs(4*i-3,1:2)];
    plot(ppk(:,1),ppk(:,2),'-g','LineWidth',2);
    plot(acr1_obs(4*i-3:4*i,3),acr1_obs(4*i-3:4*i,4),'-g','LineWidth',2,'Linestyle','- -');
    ppk=[acr1_obs(4*i,3:4);acr1_obs(4*i-3,3:4)];
    plot(ppk(:,1),ppk(:,2),'-g','LineWidth',2,'Linestyle','- -');
end

for i=1:k_att
    plot(att1_obs(4*i-3:4*i,1),att1_obs(4*i-3:4*i,2),'-y','LineWidth',2);
    ppk=[att1_obs(4*i,1:2);att1_obs(4*i-3,1:2)];
    plot(ppk(:,1),ppk(:,2),'-y','LineWidth',2);
    plot(att1_obs(4*i-3:4*i,3),att1_obs(4*i-3:4*i,4),'-y','LineWidth',2,'Linestyle','- -');
    ppk=[att1_obs(4*i,3:4);att1_obs(4*i-3,3:4)];
    plot(ppk(:,1),ppk(:,2),'-y','LineWidth',2,'Linestyle','- -');
     plot(att1_obs(4*i-3:4*i,5),att1_obs(4*i-3:4*i,6),'-y','LineWidth',2,'Linestyle','-.');
    ppk=[att1_obs(4*i,5:6);att1_obs(4*i-3,5:6)];
    plot(ppk(:,1),ppk(:,2),'-y','LineWidth',2,'Linestyle','-.');
end

path= [ps;pathxy1;pe];
 %plot(path(:,1)*1000,path(:,2)*1000,'-ok','LineWidth',2);
% plot(paths(:,1),paths(:,2),'-k','LineWidth',2);
% plot(pathe(:,1),pathe(:,2),'-k','LineWidth',2);
%set(gca,'XTick',0:0.5:5)                      %设置坐标轴范围、间距
%set(gca,'YTick',0:0.5:5)
axis equal 
% axis([0,8000,0,7000])
% set(gca,'xticklabel',mod(0:1:8,9));
% set(gca,'yticklabel',mod(0:1:7,8));

[ pathstr,path_dis] = path_straight( path,map_border,obs_border,obs_height );
end