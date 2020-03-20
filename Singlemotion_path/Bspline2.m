function [ p_s ] = Bspline2(P)
% �����������͵�B�������ߣ���Ҫǰ������������.m�ļ�
%clear all;
%���ƶ���
% P = [9.036145, 21.084337, 37.607573, 51.893287, 61.187608;
%     51.779661, 70.084746, 50.254237, 69.745763, 49.576271];
P=[P(:,1)';P(:,2)'];
 n = size(P,2)-1;
%n = 6;
k = 3;
 if size(P,2)<k+1
     h=msgbox('���Ƶ���Ŀ����','warn');
     uiwait(h,3);
      return;
 end
flag = 2;
% flag = 1�����ƾ���B��������
% flag = 2, ����׼����B��������
% flag = 3, ���Ʒֶ�Bezier����
 
switch flag
    case 1
        NodeVector = linspace(0, 1, n+k+2); % ����B�����Ľڵ�ʸ��
 
        % ������������
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
        NodeVector = U_quasi_uniform(n, k); % ׼����B�����Ľڵ�ʸ��
        p_s = DrawSpline(n, k, P, NodeVector);
    case 3
        NodeVector = U_piecewise_Bezier(n, k);  % �ֶ�Bezier���ߵĽڵ�ʸ��
        p_s = DrawSpline(n, k, P, NodeVector);
    otherwise
        fprintf('error!\n');
end

end

% U_quasi_uniform.m�ļ�
function NodeVector = U_quasi_uniform(n, k)
% ׼����B�����Ľڵ��������㣬��n+1�����ƶ��㣬k��B����
NodeVector = zeros(1, n+k+2);
piecewise = n - k + 1;       % ���ߵĶ���
if piecewise == 1       % ֻ��һ������ʱ��n = k
    for i = n+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;       % ��ֹһ������ʱ
    while flag ~= piecewise
        NodeVector(1, k+1+flag) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;
end
end

% U_piecewise_Bezier.m�ļ�
function NodeVector = U_piecewise_Bezier(n, k)
% �ֶ�Bezier���ߵĽڵ��������㣬��n+1�����ƶ��㣬k��B����
% �ֶ�Bezier�˽ڵ��ظ���Ϊk+1���ڼ�ڵ��ظ���Ϊk,������n/kΪ������
 
if ~mod(n, k) && (~mod(k, 1) && k>=1)   % ����n��k����������kΪ������
    NodeVector = zeros(1, n+k+2);   % �ڵ�ʸ������Ϊn+k+2
    NodeVector(1, n+2 : n+k+2) = ones(1, k+1);  % �Ҷ˽ڵ���1
 
    piecewise = n / k;      % �趨�ڽڵ��ֵ
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

% DrawSpline.m�ļ�
function p_s =  DrawSpline(n, k, P, NodeVector)
% B�����Ļ�ͼ����
% ��֪n+1�����ƶ���P(i), k��B������P��2*(n+1)�������ƶ�������, �ڵ�����NodeVector
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

% BaseFunction.m�ļ�
function Nik_u = BaseFunction(i, k , u, NodeVector)
% ���������Ni,k(u),NodeVectorΪ�ڵ�����
 
if k == 0       % 0��B����
    if (u >= NodeVector(i+1)) && (u < NodeVector(i+2))
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % ֧������ĳ���
    if Length1 == 0.0       % �涨0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end
end


