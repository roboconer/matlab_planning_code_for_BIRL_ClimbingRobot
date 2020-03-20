%% ==========================================================
% 函数功能：根据PA和PB两点画三维空间中的直线
% 输入： PA - 第一点坐标向量
%       PB - 第二点坐标向量
%       clr - 直线颜色设置，字符串
%       lw - 直线宽度
% 输出：hline - 直线的句柄
function hline = plotline(PA,PB,clr,lw)

    hline = plot3([PA(1);PB(1)],[PA(2);PB(2)],[PA(3);PB(3)],clr,'Linewidth',lw);
    hold on;
    
end