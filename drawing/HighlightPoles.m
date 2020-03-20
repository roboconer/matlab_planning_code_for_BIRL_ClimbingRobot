%% ===============================================================
% 函数功能：高亮杆件中的某一部分（段），已表示可夹持区域等
% 输入：pole - 杆件，n*8，一行为一根杆件
%       t - 高亮显示的部分，没两个相邻的元素标识高亮的线段
%           由于每一根线需要高亮的数量可能不同，为m*n cell数组，m为杆件数目，n/2为高亮的线段数目
%       clr - 高亮的颜色，默认为绿色
% 输出：无
function HighlightPoles(pole,t,clr)

if nargin < 3
    clr = 'g';
    pR = 30;  % 杆件半径
else
    pR = 29.5;
end

poleNum = size(pole,1);

% pR = 30;  % 杆件半径
for i = 1:poleNum
    
    % 取出杆件
    aPole = pole(i,:);
    % 单位方向向量
    pDir = (aPole(4:6)-aPole(1:3))/norm(aPole(4:6)-aPole(1:3));
    
    % 可能不止一段
    if iscell(t)
        th = t{i,:};
    else
        th = t(i,:);
    end
    
    for j = 1:2:size(th,2)
        
        % 起点和终点
        pBeg = aPole(1:3) + th(j) * pDir;
        pEnd = aPole(1:3) + th(j+1) * pDir;
        
        % 高亮显示
        if aPole(7)
            if th(j) > th(j+1)   % 或者将th排序，再计算起始和终止点，否则出现旋转角度不同的问题
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