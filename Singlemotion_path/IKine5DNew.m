%% ==========================================================
% 函数功能：根据机器人末端位姿齐次矩阵（在机器人基座标系下）求解机器人关节角
%           强制机器人的末端满足构型约束
% 输入： EETrMtx - 机器人末端位姿齐次矩阵
%       CurConfigs - 机器人当前的关节角，5维
%	jpos_G2[0] = - gdJPos[4];
%	jpos_G2[1] = gdJPos[3];
%	jpos_G2[2] = gdJPos[2];
%	jpos_G2[3] = gdJPos[1];
%	jpos_G2[4] = -gdJPos[0];
%
% 输出： Configs - 表示机器人5个关节角的向量
function [Flag, Configs] = IKine5DNew(robot)

%Q:怎么规定连杆序号的？
%A:反正对称,应该base是1
l01 = robot.LinksLen(1);
l2 = robot.LinksLen(2);
l3 = robot.LinksLen(3);
l45 = robot.LinksLen(4);

CurConfigs = deg2rad(robot.CurJAs);
% 四组解均存在的情况，1为存在
Sol_Flag = ones(1,4);
Configs = NaN * ones(1,5);  % 最后解输出
theta = ones(4,5);     % 4组逆解的暂存空间

% 不同夹持端处理（加了个断点，发现不会进else？）
if robot.FixedGripper == 1
    EETrMtx = robot.TarGraspMtx;
else
    Trx = [1 0 0 0;0 -1 0 0; 0 0 -1 0; 0 0 0 1];
    EETrMtx = Trx / robot.TarGraspMtx * Trx;
end

% 先求解theta1
% 求解theta1
theta(1:2,1) = ClampJAngle(CurConfigs(1),atan2(EETrMtx(2,4),EETrMtx(1,4)));
% 另一解
theta(3:4,1) = ClampJAngle(CurConfigs(1),atan2(-EETrMtx(2,4),-EETrMtx(1,4)));

% 测试夹持方向是否在机器人构型平面内（吸附不用判断）
vdot = dot(EETrMtx(1:3,3),[-EETrMtx(2,4),EETrMtx(1,4),0]);
% if abs(vdot) > 1e-12
%     EETrMtx(1:3,1:3) = modifyOriMtx(EETrMtx(1:3,4),EETrMtx(1:3,1));
% end

% 变换到腕关节求解
EETrMtx(1:3,4) = EETrMtx(1:3,4) - EETrMtx(1:3,3) * l45;

% 求解theta5 和 theta3
for i = 1:2
    
    s1 = sin(theta(2*i-1,1));
    c1 = cos(theta(2*i-1,1));

    % 注意 si表示从角度直接求出的正弦值，si_表示从位姿矩阵间接求出的正弦值
    s5_ = s1 * EETrMtx(1,1) - c1 * EETrMtx(2,1);
    c5_ = s1 * EETrMtx(1,2) - c1 * EETrMtx(2,2);
    
    % theta5
    theta(2*i-1:2*i,5) = ClampJAngle(CurConfigs(5),atan2(s5_,c5_));
    
    % theta3
    c3_ = ((EETrMtx(1,4)*c1 + EETrMtx(2,4)*s1)^2 + (EETrMtx(3,4) - l01)^2 - (l2^2+l3^2))/(2*l2*l3);
    s3_sq = 1 - c3_^2;
    
    if (abs(s3_sq) < 1e-12)   %等于零
        s3_ = 0; 
    elseif (s3_sq < 0)       %小于零，无解
        Sol_Flag(2*i-1:2*i) = 0;
        continue;
    else
        s3_ = sqrt(s3_sq);
    end
    
    %/////这是New版本的
    theta(2*i-1,3) = ClampJAngle(CurConfigs(3),atan2(s3_,c3_));
    % 另一种解,怎么把两个值赋给同一个内存空间！！
    theta(2*i,3) = ClampJAngle(CurConfigs(3),atan2(-s3_,c3_));
    
%     %/////这是22版本的
%         theta(2*i-1,3) = ClampJAngle(CurConfigs(3),atan2(s3_,c3_));
%      if theta(2*i-1,3) <-pi
%          theta(2*i-1,3) = theta(2*i-1,3) +2*pi;
%      elseif  theta(2*i-1,3) >pi
%          theta(2*i-1,3) = theta(2*i-1,3) -2*pi;
%      end
%     % 另一种解,怎么把两个值赋给同一个内存空间！！
%     theta(2*i,3) = ClampJAngle(CurConfigs(3),atan2(-s3_,c3_));
%     if theta(2*i,3) <-pi
%          theta(2*i,3) = theta(2*i-1,3) +2*pi;
%      elseif  theta(2*i,3) >pi
%          theta(2*i,3) = theta(2*i-1,3) -2*pi;
%      end
end

if ~Sol_Flag(1) && ~Sol_Flag(3)   %腰关节确定无解
    Flag = 0;
    return;
end
    
% 求解theta2 和 theta4
for i = 1:4
    
    if Sol_Flag(i)
        
        s1 = sin(theta(i,1));
        c1 = cos(theta(i,1));
        
        s3 = sin(theta(i,3));
        c3 = cos(theta(i,3));
        
        % theta2
        pzl01 =  EETrMtx(3,4) - l01;
        pxpy = EETrMtx(1,4) * c1 + EETrMtx(2,4) * s1;        
        s2_ = pzl01 * (l3 * c3 + l2) - pxpy * l3 * s3;  % 分母相等且为正，可都不计算
        c2_ = pxpy * (l3 * c3 + l2) + pzl01 * l3 * s3;  % 分母相等且为正，可都不计算
        
        theta(i,2) = ClampJAngle(CurConfigs(2),atan2(s2_,c2_));
        
        % theta4
        s234_ = EETrMtx(1,3) * c1 + EETrMtx(2,3) * s1;
        c234_ = - EETrMtx(3,3);
        
        theta234 = atan2(s234_,c234_);  %234之和
        theta(i,4) = ClampJAngle(CurConfigs(4), theta234 - theta(i,2) - theta(i,3));
        
    end
end

% 关节角限位
JiontLimitUp = deg2rad([360,240,140,240,360]);
JiontLimitLow = deg2rad([-360,-60,-140,-60,-360]);

% 选择最佳结果
criterion = ones(1,4);      %可以用inf*ones(1,4)进一步防止错误出现
result = 0;

for i = 1:4
    
    if Sol_Flag(i)       
        % 是否为Nan元素
        if find(isnan(theta(i,:))==1,1)
            Sol_Flag(i) = 0;
            continue;
        end
    end
    if Sol_Flag(i)   % 有解的才进行关节角检查
        % 是否超出限位
        if any(theta(i,:) > JiontLimitUp) || any(theta(i,:) < JiontLimitLow)      % 超出限位
            Sol_Flag(i) = 0;
            continue; % 提前结束限位检查
        end
    end
%%2020.3.16注释byCH    
%     if Sol_Flag(i)
%         % 这个度量函数不是最合适的？
%         criterion(i) = abs(theta(i,1)-CurConfigs(1)) + ...
%             abs(theta(i,2)-CurConfigs(2)) + ...
%             abs(theta(i,3)-CurConfigs(3)) + ...
%             abs(theta(i,4)-CurConfigs(4));
%             %abs(theta(i,5)-CurConfigs(5));
%         if ~result  %第一次得到有效解，先进行初始化
%             result = i;
%             best = criterion(i);
%         elseif best > criterion(i)      %第二次以及以后，对有效解进行比较
%             result = i;                 %更优的话就替换掉原有解
%             best = criterion(i);
%         elseif abs(best - criterion(i))<1e-5
%             %2020.3.15 修改，原来为：% if theta(i,2)>0   %偏向往上凸的构型，即第二个摆转关节逆向摆
%             if theta(i,3)<0        
%                  result = i;                
%             best = criterion(i);
%             else
%                continue; 
%             end
%         end
%     end
    if Sol_Flag(i)
        % 这个度量函数不是最合适的？
        criterion(i) = abs(theta(i,1)-CurConfigs(1)) + ...
            abs(theta(i,2)-CurConfigs(2)) + ...
            abs(theta(i,3)-CurConfigs(3)) + ...
            abs(theta(i,4)-CurConfigs(4));
            %abs(theta(i,5)-CurConfigs(5));
%          if ( theta(i,3) >= 0 ) && (theta(i, 2) >= 0 )  
         if theta(i,3)<=0  
            if ~result  %第一次得到有效解，先进行初始化
                result = i;
                best = criterion(i);
                      
            elseif best > criterion(i)      %第二次以及以后，对有效解进行比较
                result = i;                 %更优的话就替换掉原有解
                best = criterion(i);
            else
               continue; 
            end
        end
    end
    
end

if result
    Flag = 1;
    Configs(1,:) = rad2deg(theta(result,:));     %选择最优的解进行输出
else
    Flag = 0;
end

end


% 根据当前角度优化计算转角，限定在[-360,+360]内
function betterJA = ClampJAngle(CurJA,TarJA)

dPI = 2*pi;

if TarJA > dPI
    betterJA = TarJA - floor(TarJA/dPI)*dPI;
elseif TarJA < - dPI
    TarJA = TarJA + dPI;
    betterJA = TarJA - floor(TarJA/dPI)*dPI-dPI;
elseif TarJA >= 0
    if (abs(TarJA - CurJA) > abs(TarJA - CurJA - dPI))
        betterJA = TarJA - dPI;
    else
        betterJA = TarJA;
    end
else
    if (abs(TarJA - CurJA) > abs(TarJA - CurJA + dPI))
        betterJA = TarJA + dPI;
    else
        betterJA = TarJA;
    end
end

end

% 修改夹持方向，使5DoFs Climbot能到达
% 也可以理解为以保证位置和杆向（姿态矩阵的n列）为根本
function OriMtx = modifyOriMtx(Pos,n)

% OriMtx = [n|o|a]

% 计算夹持方向
if abs(n(3)) > 1e-12
    alpha = atan2(Pos(2),Pos(1));
    tanA = tan(alpha);
    lambda = -(n(1)+n(2)*tanA)/n(3);
    a = n; % 初始化
    if Pos(1) > 0
        a(1) = sqrt(1 / (1 + tanA^2 + lambda^2));
    else
        a(1) = -sqrt(1 / (1 + tanA^2 + lambda^2));
    end
    a(2) = a(1) * tanA;
    a(3) = lambda * a(1);
else
    a = [0;0;-1];
end
o = cross(a,n);

OriMtx = [n,o,a];

end
