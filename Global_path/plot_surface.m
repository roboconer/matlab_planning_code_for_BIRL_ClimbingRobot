[A,num_point,num_sur,~,~] = surfaces;
Vn=zeros(num_sur,3);
Pstart = zeros(1,3);
Pend = zeros(1,3);
r=0.2;
% for i=1:num_sur
%     v1= (A1{i}(num_point(i),:)-A1{i}(1,:))/norm(A1{i}(num_point(i),:)-A1{i}(1,:));
%     v2= (A1{i}(2,:)-A1{i}(1,:))/norm(A1{i}(2,:)-A1{i}(1,:));
%     ang_cos = dot(v1,v2);
%     ang_sin = sqrt(1-ang_cos^2);
%     l = r/ang_sin;
%     A{i}(1,:) = A1{i}(1,:)+ v1*l +v2*l;
%     
%     v1=( A1{i}(1,:)-A1{i}(num_point(i),:))/norm(A1{i}(1,:)-A1{i}(num_point(i),:));
%     v2= (A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:))/norm(A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:));
%     ang_cos = dot(v1,v2);
%     ang_sin = sqrt(1-ang_cos^2);
%     l = r/ang_sin;
%     A{i}(num_point(i),:) = A1{i}(num_point(i),:)+ v1*l +v2*l;
%     
%    for j=2:num_point(i)-1
%        v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
%        v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
%        ang_cos = dot(v1,v2);
%        ang_sin = sqrt(1-ang_cos^2);
%        l = r/ang_sin;
%        A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;
%        
%    end
% end
%set(0,'defaultfigurecolor','w')          %设置背景为白色
% print 8.jpeg -djpeg -r300            %打印输出图片，先在打印预览设置自动选择大小
% print 8.png -dpng -r600
for i= 1:num_sur
    A{i} = A{i}*1000;
end
%color=[7 6 5 3 2];
for i=1:num_sur
   pp = patch(A{i}(:,1),A{i}(:,2),A{i}(:,3),[0.2,0.7,0.2],'facealpha',0.5,'LineWidth',1);
   % pp = patch(A{i}(:,1)+1000,A{i}(:,2)-1000,A{i}(:,3),50+15*i,'facealpha',0.5);
   % pp.FaceColor='blue';
    hold on
end

for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm*600;
    %quiver3(A{i}(2,1),A{i}(2,2),A{i}(2,3),Vn(i,1),Vn(i,2),Vn(i,3),'k','filled','LineWidth',1,'maxheadsize',1.3);
    mArrow3([A{i}(2,1),A{i}(2,2),A{i}(2,3)],[Vn(i,1),Vn(i,2),Vn(i,3)]+[A{i}(2,1),A{i}(2,2),A{i}(2,3)],'color','m','stemWidth',25,'facealpha',0.7);
end


% for i =1: num_point(1)
%     Pstart = Pstart+A{1}(i,:);
% end
% Pstart = Pstart/num_point(1);
% for i =1: num_point(6)
%     Pend = Pend+ A{6}(i,:);
% end
% Pend = Pend/num_point(6);
%  plot3(Pstart(1,1),Pstart(1,2),Pstart(1,3),'-k*','LineWidth',1);
% plot3(Pend(1,1),Pend(1,2),Pend(1,3),'-k*','LineWidth',1);

axis equal 
% 
% set(gca,'XTick',0:2000:10000)                      %设置坐标轴范围、间距
% set(gca,'YTick',2000:2000:10000)
% set(gca,'ZTick',2000:2000:10000)
% axis([0,10000,0,10000,0,10000])
% set(gca,'xticklabel',mod(0:2:10,12));
% set(gca,'yticklabel',mod(2:2:10,12));
% set(gca,'zticklabel',mod(2:2:10,12));
% set(gca,'XTick',0:1000:5000)
%  set(gca,'YTick',1000:1000:5000)
%  set(gca,'ZTick',1000:1000:5000)
%  axis([0,5000,0,5000,0,5000])
% set(gca,'xticklabel',mod(0:1:5,6));
% set(gca,'yticklabel',mod(1:1:5,6));
% set(gca,'zticklabel',mod(1:1:5,6));
fz =17;
hxl= xlabel('\it{X}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');        %设置坐标轴格式
%set(hxl,'Position',[1000 -1500 0]);
set(hxl,'Position',[500 -1100 0]);
hyl= ylabel('\it{Y}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');
%set(hyl,'Position',[11200 0 0]);
set(hyl,'Position',[-1000 500 0]);
hzl= zlabel('\it{Z}\rm/\it{m}','FontSize',fz,'FontName','Times New Roman');
%set(hzl,'Position',[-500 -1000 5000])
set(hzl,'Position',[-500 6000 2500]);
set(gca,'Fontname','Times New Roman');
set(gca,'fontsize',fz);

viewmtx(47.5,20);
DrawRobotDemo;
%grid on