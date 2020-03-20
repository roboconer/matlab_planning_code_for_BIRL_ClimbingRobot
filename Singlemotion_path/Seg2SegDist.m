%% ===================================================================
% ！！！ 有待改善之处：利用MATLAB矩阵运算的特点，改成一次可以计算多根线段之间的距离的
% 函数功能：求两根线段之间的最小距离
% 输入： Seg1 - 线段1,两个端点表示 S1(s)=P1+s*(Q1-P1)
%       Seg2 - 线段2,两个端点表示 S2(t)=P2+t*(Q2-P2)
%
% 输出： dist - 最小距离
%       CPtSeg1 - 线段1上对应最小距离的点  Closest Point at Segment 1
%       CPtSeg2 - 线段2上对应最小距离的点
function [dist,CPtSeg1,CPtSeg2] = Seg2SegDist(Seg1,Seg2)

EPSILON = 1e-5;

%提取线段端点
p1 = Seg1(1,:); q1 = Seg1(2,:);
p2 = Seg2(1,:); q2 = Seg2(2,:);

%计算方向向量
d1 = q1 - p1; % Direction vector of segment S1
d2 = q2 - p2; % Direction vector of segment S2
r = p1 - p2;
a = dot(d1, d1); % Squared length of segment S1, always nonnegative
e = dot(d2, d2); % Squared length of segment S2, always nonnegative
f = dot(d2, r);

% Check if either or both segments degenerate into points
if (a <= EPSILON && e <= EPSILON)
    % Both segments degenerate into points
    %s = 0.0; t = 0.0;
    CPtSeg1 = p1;
    CPtSeg2 = p2;
    dist = dot(CPtSeg1 - CPtSeg2, CPtSeg1 - CPtSeg2);
    return;
end

if (a <= EPSILON)
    % First segment degenerates into a point
    s = 0.0;
    t = f / e; % s = 0 => t = (b*s + f) / e = f / e
    t = Clamp(t, 0.0, 1.0);
else
    c = dot(d1, r);
    if (e <= EPSILON)
        % Second segment degenerates into a point
        t = 0.0;
        s = Clamp(-c / a, 0.0, 1.0); % t = 0 => s = (b*t - c) / a = -c / a
    else
        % The general nondegenerate case starts here
        b = dot(d1, d2);
        denom = a*e-b*b; % Always nonnegative
        
        % If segments not parallel, compute closest point on L1 to L2, and
        % clamp to segment S1. Else pick arbitrary s (here 0)
        if (denom ~= 0.0)
            s = Clamp((b*f - c*e) / denom, 0.0, 1.0);
        else
            s = 0.0;
        end
        
        % Compute point on L2 closest to S1(s) using
        % t = dot((P1+D1*s)-P2,D2) / dot(D2,D2) = (b*s + f) / e
        t = (b*s + f) / e;
        
        % If t in [0,1] done. Else clamp t, recompute s for the new value
        % of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
        % and clamp s to [0, 1]
        if (t < 0.0)
            t = 0.0;
            s = Clamp(-c / a, 0.0, 1.0);
        elseif (t > 1.0)
            t = 1.0;
            s = Clamp((b - c) / a, 0.0, 1.0);
        end
    end
end

CPtSeg1 = p1 + d1 * s;
CPtSeg2 = p2 + d2 * t;
dist = sqrt(dot(CPtSeg1 - CPtSeg2, CPtSeg1 - CPtSeg2));

end


%% 辅助函数
% Clamp n to lie within the range [min, max]
function N = Clamp(n,min,max)

if n < min
    N = min;
    return;
elseif n > max
    N = max;
    return;
else
    N = n;
end;

end