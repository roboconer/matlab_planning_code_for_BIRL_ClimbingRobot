% for i = 1:size(wall,2)
%     pp = patch(wall{1,i}(:,1),wall{1,i}(:,2),wall{1,i}(:,3),[0.2,0.7,0.2],'facealpha',0.5,'LineWidth',1);
%     hold on
% end

% for i = 1:size(A1,2)
%     pp = patch(A1{1,i}(:,1),A1{1,i}(:,2),A1{1,i}(:,3),[0.2,0.7,0.2],'facealpha',0.5,'LineWidth',1);
%     hold on
% end

% pp = patch(poly(:,1),poly(:,2),poly(:,3),[0.2,0.7,0.2],'facealpha',0.5,'LineWidth',1);
% hold on

% quiver3(mean_p(1),mean_p(2),mean_p(3),vec(1),vec(2),vec(3),1)
% hold on

quiver3(point(1),point(2),point(3),v2(1),v2(2),v2(3),1)
hold on