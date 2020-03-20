[A,num_point,num_sur] = surfaces;
Vn=zeros(num_sur,3);
for i= 1:num_sur
    A{i} = A{i}*1000;
end
%color=[7 6 5 3 2];
for i=1:num_sur
    pp = patch(A{i}(:,1),A{i}(:,2),A{i}(:,3),50+15*i,'facealpha',0.7);
   % pp.FaceColor='blue';
    hold on
end

for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm*500;
    quiver3(A{i}(2,1),A{i}(2,2),A{i}(2,3),Vn(i,1),Vn(i,2),Vn(i,3),'k','filled','LineWidth',1,'maxheadsize',1.3);
end
axis equal 
set(gca,'XTick',0:2000:10000)
set(gca,'YTick',2000:2000:10000)
set(gca,'ZTick',2000:2000:10000)

axis([0,10000,0,10000,0,10000])
fz =14;
hxl= xlabel('\it{x}\rm(\it{m}\rm)','FontSize',fz,'FontName','Times New Roman');
set(hxl,'Position',[1000 -1000 0]);
hyl= ylabel('\it{y}\rm(\it{m}\rm)','FontSize',fz,'FontName','Times New Roman');
set(hyl,'Position',[-1000 1000 0]);
hzl= zlabel('\it{z}\rm(\it{m}\rm)','FontSize',fz,'FontName','Times New Roman');
set(hzl,'Position',[-1000 11000 5000]);
set(gca,'Fontname','Times New Roman');
set(gca,'fontsize',fz);
DrawRobotDemo;