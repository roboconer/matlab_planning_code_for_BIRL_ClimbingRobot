%% ==========================================================
% �������ܣ�����PA��PB���㻭��ά�ռ��е�ֱ��
% ���룺 PA - ��һ����������
%       PB - �ڶ�����������
%       clr - ֱ����ɫ���ã��ַ���
%       lw - ֱ�߿��
% �����hline - ֱ�ߵľ��
function hline = plotline(PA,PB,clr,lw)

    hline = plot3([PA(1);PB(1)],[PA(2);PB(2)],[PA(3);PB(3)],clr,'Linewidth',lw);
    hold on;
    
end