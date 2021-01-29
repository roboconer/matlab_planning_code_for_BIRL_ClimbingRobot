[A,num_point,num_sur,Pstart,Pend] = surfaces;
Vn=zeros(num_sur,3);
% Pstart = zeros(1,3);
% Pend = zeros(1,3);
r=0.2;
input_map2;
for i= 1:num_sur
    A{i} = A{i}*1000;
end
%color=[7 6 5 3 2];
for i=1:num_sur
   pp = patch(A{i}(:,1),A{i}(:,2),A{i}(:,3),[0.467,0.533,0.6],'facealpha',0.5,'LineWidth',1);
   % pp = patch(A{i}(:,1)+1000,A{i}(:,2)-1000,A{i}(:,3),50+15*i,'facealpha',0.5);
   % pp.FaceColor='blue';
    hold on
end

for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm*600;
    %quiver3(A{i}(2,1),A{i}(2,2),A{i}(2,3),Vn(i,1),Vn(i,2),Vn(i,3),'k','filled','LineWidth',1,'maxheadsize',1.3);
    mArrow3([A{i}(2,1),A{i}(2,2),A{i}(2,3)],[Vn(i,1),Vn(i,2),Vn(i,3)]*0.5+[A{i}(2,1),A{i}(2,2),A{i}(2,3)],'color','m','stemWidth',15,'facealpha',0.7);
end

axis equal 
% 
% set(gca,'XTick',0:2000:10000)                      %设置坐标轴范围、间距
% set(gca,'YTick',2000:2000:10000)
% set(gca,'ZTick',2000:2000:10000)
% axis([0,10000,0,10000,0,10000])
% set(gca,'xticklabel',mod(0:2:10,12));
% set(gca,'yticklabel',mod(2:2:10,12));
% set(gca,'zticklabel',mod(2:2:10,12));
set(gca,'XTick',0:1000:3000)
 set(gca,'YTick',-1500:1000:-500)
 set(gca,'ZTick',0:1000:2500)
 axis([0,3000,-1500,0, 0,2500])
set(gca,'xticklabel',mod(0:1:4,5));
set(gca,'yticklabel',mod(1:1:3,4));
set(gca,'zticklabel',mod(1:1:4,5));
fz =17;
%hxl= xlabel('\it{X}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');        %设置坐标轴格式
%set(hxl,'Position',[1000 -1500 0]);
%set(hxl,'Position',[500 -1100 0]);
%hyl= ylabel('\it{Y}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');
%set(hyl,'Position',[1100 0 0]);
%set(hyl,'Position',[-1000 500 0]);
%hzl= zlabel('\it{Z}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');
%set(hzl,'Position',[-500 -1000 5000])
%set(hzl,'Position',[-500 6000 2500]);
set(gca,'Fontname','Times New Roman');
set(gca,'fontsize',fz);
[A1,num_point,num_sur,~,~] = surfaces;
viewmtx(47.5,20);
r=0.1998;   %吸盘半径

Vz=zeros(num_sur,3);
Vy=zeros(num_sur,3);
Vx=zeros(num_sur,3);
pw = zeros(num_sur,3);
psw= zeros(num_sur,3);
% Pstart = zeros(1,3);
% Pend = zeros(1,3);
R_sw = zeros(3,3,num_sur);%平面坐标系在世界坐标系下的转换矩阵
R_ws = zeros(3,3,num_sur);%世界坐标系在平面坐标系下的转换矩阵
Xa=[1,0,0];
Ya=[0,1,0];
Za=[0,0,1];

for i=1:num_sur
    v1= (A1{i}(num_point(i),:)-A1{i}(1,:))/norm(A1{i}(num_point(i),:)-A1{i}(1,:));
    v2= (A1{i}(2,:)-A1{i}(1,:))/norm(A1{i}(2,:)-A1{i}(1,:));
    ang_cos = dot(v1,v2);
    ang_sin = sqrt(1-ang_cos^2);
    l = r/ang_sin;
    A{i}(1,:) = A1{i}(1,:)+ v1*l +v2*l;
    
    v1=( A1{i}(1,:)-A1{i}(num_point(i),:))/norm(A1{i}(1,:)-A1{i}(num_point(i),:));
    v2= (A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:))/norm(A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:));
    ang_cos = dot(v1,v2);
    ang_sin = sqrt(1-ang_cos^2);
    l = r/ang_sin;
    A{i}(num_point(i),:) = A1{i}(num_point(i),:)+ v1*l +v2*l;
    
   for j=2:num_point(i)-1
       v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
       v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
       ang_cos = dot(v1,v2);
       ang_sin = sqrt(1-ang_cos^2);
       l = r/ang_sin;
       A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;   
   end
end
for i= 1:num_sur
    B{i} = A{i};
end
for i=1:num_sur
    Vz(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量,建立空间坐标系
    Vz_norm=norm(Vz(i,:));
    Vz(i,:)=Vz(i,:)/Vz_norm;
end
for i = 1:num_sur
    Vx(i,:) = A{i}(2,:)-A{i}(1,:);
    Vx_norm=norm(Vx(i,:));
    Vx(i,:)=Vx(i,:)/Vx_norm;
    Vy(i,:)=cross(Vz(i,:),Vx(i,:));
end
for i =1:num_sur
R_sw(:,:,i)=[dot(Vx(i,:),Xa),dot(Vy(i,:),Xa),dot(Vz(i,:),Xa);dot(Vx(i,:),Ya),dot(Vy(i,:),Ya),dot(Vz(i,:),Ya);dot(Vx(i,:),Za),dot(Vy(i,:),Za),dot(Vz(i,:),Za)]; 
R_ws(:,:,i) = R_sw(:,:,i)';
end
for i=1:num_sur
    pw(i,:) = A{i}(1,:);
    psw(i,:) =(-R_ws(:,:,i)*pw(i,:)')';  %世界坐标系的原点在平面坐标系上的表示
end

for i=1:num_sur
    for j=1:num_point(i)
        B{i}(j,:) = (R_ws(:,:,i)*A{i}(j,:)')'+psw(i,:);
    end
end

 for i=1:num_sur
  
 map_border=zeros(num_point(i),2);
for j=1:num_point(i)
    map_border(j,:) = B{i}(j,1:2);
end

%  plot_obs( map_border,Obs_border{i},Obs_height{i},R_sw(:,:,i),pw(i,:) );
 end
 
%  proute(1,:)= Pstart;
% 
% for i=2:(2*num_sur-1)
%     proute(i,:) = x(i-1,:);
% end
% proute(2*num_sur,:) =  Pend;
%     proute=proute*1000;
% plot3(proute(:,1)',proute(:,2)',proute(:,3)','-k*','LineWidth',1);
% hold on 
% for i=1:num_sur
%     mArrow3([proute(2*i-1,1)+(proute(2*i,1)-proute(2*i-1,1))*0.3,proute(2*i-1,2)+(proute(2*i,2)-proute(2*i-1,2))*0.3,proute(2*i-1,3)+(proute(2*i,3)-proute(2*i-1,3))*0.3],[proute(2*i-1,1)+(proute(2*i,1)-proute(2*i-1,1))*0.7,proute(2*i-1,2)+(proute(2*i,2)-proute(2*i-1,2))*0.7,proute(2*i-1,3)+(proute(2*i,3)-proute(2*i-1,3))*0.7],'color','k','stemWidth',20,'facealpha',0.7);
%     %quiver3(proute(2*i-1,1),proute(2*i-1,2),proute(2*i-1,3),proute(2*i,1)-proute(2*i-1,1),proute(2*i,2)-proute(2*i-1,2),proute(2*i,3)-proute(2*i-1,3),0.5,'k','filled','LineWidth',2,'maxheadsize',1.2)
% end
% DrawRobotDemo;
% grid on