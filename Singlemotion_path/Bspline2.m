function [ p_s ] = Bspline2(P)
% 绘制三种类型的B样条曲线，需要前面所给的所有.m文件
%clear all;
%控制顶点
% P = [9.036145, 21.084337, 37.607573, 51.893287, 61.187608;
%     51.779661, 70.084746, 50.254237, 69.745763, 49.576271];
P=[P(:,1)';P(:,2)'];
 n = size(P,2)-1;
%n = 6;
k = 3;
 if size(P,2)<k+1
     h=msgbox('控制点数目不够','warn');
     uiwait(h,3);
      return;
 end
flag = 2;
% flag = 1，绘制均匀B样条曲线
% flag = 2, 绘制准均匀B样条曲线
% flag = 3, 绘制分段Bezier曲线
 
switch flag
    case 1
        NodeVector = linspace(0, 1, n+k+2); % 均匀B样条的节点矢量
 
        % 绘制样条曲线
        plot(P(1, 1:n+1), P(2, 1:n+1),...
                        'o','LineWidth',1,...
                        'MarkerEdgeColor','k',...
                        'MarkerFaceColor','g',...
                        'MarkerSize',6);
        line(P(1, 1:n+1), P(2, 1:n+1));
        Nik = zeros(n+1, 1);
        for u = k/(n+k+1) : 0.001 : (n+1)/(n+k+1)
            % for u = 0 : 0.005 : 1
            for i = 0 : 1 : n
                Nik(i+1, 1) = BaseFunction(i, k , u, NodeVector);
            end
        p_u = P * Nik;
        line(p_u(1,1), p_u(2,1), 'Marker','.','LineStyle','-', 'Color',[.3 .6 .9]);
        end
    case 2
        NodeVector = U_quasi_uniform(n, k); % 准均匀B样条的节点矢量
        p_s = DrawSpline(n, k, P, NodeVector);
    case 3
        NodeVector = U_piecewise_Bezier(n, k);  % 分段Bezier曲线的节点矢量
        p_s = DrawSpline(n, k, P, NodeVector);
    otherwise
        fprintf('error!\n');
end

end

% U_quasi_uniform.m文件
function NodeVector = U_quasi_uniform(n, k)
% 准均匀B样条的节点向量计算，共n+1个控制顶点，k次B样条
NodeVector = zeros(1, n+k+2);
piecewise = n - k + 1;       % 曲线的段数
if piecewise == 1       % 只有一段曲线时，n = k
    for i = n+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;       % 不止一段曲线时
    while flag ~= piecewise
        NodeVector(1, k+1+flag) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;
end
end

% U_piecewise_Bezier.m文件
function NodeVector = U_piecewise_Bezier(n, k)
% 分段Bezier曲线的节点向量计算，共n+1个控制顶点，k次B样条
% 分段Bezier端节点重复度为k+1，内间节点重复度为k,且满足n/k为正整数
 
if ~mod(n, k) && (~mod(k, 1) && k>=1)   % 满足n是k的整数倍且k为正整数
    NodeVector = zeros(1, n+k+2);   % 节点矢量长度为n+k+2
    NodeVector(1, n+2 : n+k+2) = ones(1, k+1);  % 右端节点置1
 
    piecewise = n / k;      % 设定内节点的值
    Flg = 0;
    if piecewise > 1
        for i = 2 : piecewise
            for j = 1 : k
                NodeVector(1, k+1 + Flg*k+j) = (i-1)/piecewise;
            end
            Flg = Flg + 1;
        end
    end
 
else
    fprintf('error!\n');
end
end

% DrawSpline.m文件
function p_s =  DrawSpline(n, k, P, NodeVector)
% B样条的绘图函数
% 已知n+1个控制顶点P(i), k次B样条，P是2*(n+1)矩阵存控制顶点坐标, 节点向量NodeVector
plot(P(1, 1:n+1), P(2, 1:n+1),...
                    'o','LineWidth',1,...
                    'MarkerEdgeColor','k',...
                    'MarkerFaceColor','g',...
                    'MarkerSize',6);
line(P(1, 1:n+1), P(2, 1:n+1));
Nik = zeros(n+1, 1);
 num_p = 1;
for u = 0 : 0.005 : 1-0.005
    for i = 0 : 1 : n
        Nik(i+1, 1) = BaseFunction(i, k , u, NodeVector);
    end
    p_u = P * Nik;
    p_s(num_p,:) = p_u;
    num_p = num_p+1;
    if u == 0
        tempx = p_u(1,1);
        tempy = p_u(2,1);
        line([tempx p_u(1,1)], [tempy p_u(2,1)],...
            'Marker','.','LineStyle','-', 'Color',[.3 .6 .9], 'LineWidth',3);
    else
        line([tempx p_u(1,1)], [tempy p_u(2,1)],...
            'Marker','.','LineStyle','-', 'Color',[.3 .6 .9], 'LineWidth',3);
        tempx = p_u(1,1);
        tempy = p_u(2,1);
    end
end

end

% BaseFunction.m文件
function Nik_u = BaseFunction(i, k , u, NodeVector)
% 计算基函数Ni,k(u),NodeVector为节点向量
 
if k == 0       % 0次B样条
    if (u >= NodeVector(i+1)) && (u < NodeVector(i+2))
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % 支撑区间的长度
    if Length1 == 0.0       % 规定0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end
end


