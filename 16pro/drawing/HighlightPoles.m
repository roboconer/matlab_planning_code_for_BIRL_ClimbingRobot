%% ===============================================================
% �������ܣ������˼��е�ĳһ���֣��Σ����ѱ�ʾ�ɼг������
% ���룺pole - �˼���n*8��һ��Ϊһ���˼�
%       t - ������ʾ�Ĳ��֣�û�������ڵ�Ԫ�ر�ʶ�������߶�
%           ����ÿһ������Ҫ�������������ܲ�ͬ��Ϊm*n cell���飬mΪ�˼���Ŀ��n/2Ϊ�������߶���Ŀ
%       clr - ��������ɫ��Ĭ��Ϊ��ɫ
% �������
function HighlightPoles(pole,t,clr)

if nargin < 3
    clr = 'g';
    pR = 30;  % �˼��뾶
else
    pR = 29.5;
end

poleNum = size(pole,1);

% pR = 30;  % �˼��뾶
for i = 1:poleNum
    
    % ȡ���˼�
    aPole = pole(i,:);
    % ��λ��������
    pDir = (aPole(4:6)-aPole(1:3))/norm(aPole(4:6)-aPole(1:3));
    
    % ���ܲ�ֹһ��
    if iscell(t)
        th = t{i,:};
    else
        th = t(i,:);
    end
    
    for j = 1:2:size(th,2)
        
        % �����յ�
        pBeg = aPole(1:3) + th(j) * pDir;
        pEnd = aPole(1:3) + th(j+1) * pDir;
        
        % ������ʾ
        if aPole(7)
            if th(j) > th(j+1)   % ���߽�th�����ټ�����ʼ����ֹ�㣬���������ת�ǶȲ�ͬ������
                drawcube(pEnd,pBeg,pR+11,aPole(8),clr); 
            else
                drawcube(pBeg,pEnd,pR+11,aPole(8),clr);
            end
        else
            drawcylinder(pBeg,pEnd,pR+1,clr,1,1);
        end
        
        hold on;
        
    end
    
end

end