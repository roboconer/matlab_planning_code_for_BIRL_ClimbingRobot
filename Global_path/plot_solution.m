function proute = plot_solution(x)
[A,num_point,num_sur,Pstart,Pend] = surfaces;
proute= zeros(num_sur*2,3);
% Pstart = zeros(1,3);
% Pend = zeros(1,3);
Vn=zeros(num_sur,3);

axis equal 
% set(gca,'Xtick',0:1:10)
% set(gca,'Ytick',0:1:10)
% set(gca,'Ztick',0:1:10)
% for i=1:num_sur
%     patch(A{i}(:,1),A{i}(:,2),A{i}(:,3),50+15*i,'facealpha',0.7);
%     hold on
% end
% 
% for i=1:num_sur
%     Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
%     Vn_norm=norm(Vn(i,:));
%     Vn(i,:)=Vn(i,:)/Vn_norm;
%     quiver3(A{i}(2,1),A{i}(2,2),A{i}(2,3),Vn(i,1),Vn(i,2),Vn(i,3),'k','filled','LineWidth',1,'maxheadsize',1.3);
% end

% for i =1: num_point(1)
%     Pstart = Pstart+A{1}(i,:);
% end
% Pstart = Pstart/num_point(1);
% for i =1: num_point(num_sur)
%     Pend = Pend+ A{num_sur}(i,:);
% end
% Pend = Pend/num_point(num_sur);

proute(1,:)= Pstart;

for i=2:(2*num_sur-1)
    proute(i,:) = x(i-1,:);
end
proute(2*num_sur,:) = Pend;
    proute=proute*1000;
plot3(proute(:,1)',proute(:,2)',proute(:,3)','-k*','LineWidth',1);
hold on 
for i=1:num_sur
    mArrow3([proute(2*i-1,1)+(proute(2*i,1)-proute(2*i-1,1))*0.3,proute(2*i-1,2)+(proute(2*i,2)-proute(2*i-1,2))*0.3,proute(2*i-1,3)+(proute(2*i,3)-proute(2*i-1,3))*0.3],[proute(2*i-1,1)+(proute(2*i,1)-proute(2*i-1,1))*0.7,proute(2*i-1,2)+(proute(2*i,2)-proute(2*i-1,2))*0.7,proute(2*i-1,3)+(proute(2*i,3)-proute(2*i-1,3))*0.7],'color','k','stemWidth',20,'facealpha',0.7);
    %quiver3(proute(2*i-1,1),proute(2*i-1,2),proute(2*i-1,3),proute(2*i,1)-proute(2*i-1,1),proute(2*i,2)-proute(2*i-1,2),proute(2*i,3)-proute(2*i-1,3),0.5,'k','filled','LineWidth',2,'maxheadsize',1.2)
end

plot_surface;

end