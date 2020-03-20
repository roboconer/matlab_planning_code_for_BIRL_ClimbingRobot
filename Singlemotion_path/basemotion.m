function [p_s,con_v] = basemotion( stp,enp,bap )
%规划双足爬壁机器人的通用单步运动路径
vst = stp - bap;
ven = enp - bap;
min_angle = 1;
cos_v = dot(vst,ven)/(norm(vst)*norm(ven));
angle_st = acos(cos_v);         %求单步运动的偏转角
r_st = (norm(vst)+norm(ven))/2;    %起始点步距和目标点步距的平均值
 vst = vst/norm(vst);
 ven = ven/norm(ven);
if abs(cos_v-1)<10^-5
    if vst(1,2)==0
        vmed = [0 1];
    else
    vmed = [1,-vst(1,1)/vst(1,2)];
    end
    vmed = vmed/norm(vmed);
else
    if min_angle ==1
    vmed = vst + ven;              %偏转角二分法求控制点
    else
        vmed = -(vst+ven);
    end
    vmed = vmed/norm(vmed);
end
% r_st = (norm(vst)+norm(ven))/2;    %起始点步距和目标点步距的平均值
% vst = vst/norm(vst);
% ven = ven/norm(ven);
% vmed = vst + ven;              %偏转角二分法求控制点
% vmed = vmed/norm(vmed);
vstm = vmed +vst;
vstm = vstm/norm(vstm)*r_st;
venm = vmed +ven;
venm = venm/norm(venm)*r_st;
vmed = vmed*r_st;
con1 = vstm + bap;
con2 = venm + bap;
con0 = vmed + bap;
con_v = [stp;con1;con0;con2;enp];
%P=[con_v(:,1)';con_v(:,2)'];
% figure(1);
% plot(bap(1,1),bap(1,2),'*');
% % set(gca,'XTick',0:1:5)                      %设置坐标轴范围、间距
% % set(gca,'YTick',1:1:5)
% % axis([-2,5,-1,5])
% 
% hold on;
% plot([stp(1,1),bap(1,1),enp(1,1)],[stp(1,2),bap(1,2),enp(1,2)],'k-','Linewidth',2);
% Bspline2(con_v);
%figure(2);
% plot(bap(1,1),bap(1,2),'*')
% set(gca,'XTick',0:1:5)                      %设置坐标轴范围、间距
% set(gca,'YTick',1:1:5)
% axis([-2,5,-1,5]);
axis equal;
hold on
%plot([stp(1,1),bap(1,1),enp(1,1)],[stp(1,2),bap(1,2),enp(1,2)],'k-','Linewidth',2);
p_s=Bspline3(con_v);
end

