%% ==========================================================
% �������ܣ����ݻ�����ĩ��λ����ξ����ڻ����˻�����ϵ�£��������˹ؽڽ�
%           ǿ�ƻ����˵�ĩ�����㹹��Լ��
% ���룺 EETrMtx - ������ĩ��λ����ξ���
%       CurConfigs - �����˵�ǰ�Ĺؽڽǣ�5ά
%	jpos_G2[0] = - gdJPos[4];
%	jpos_G2[1] = gdJPos[3];
%	jpos_G2[2] = gdJPos[2];
%	jpos_G2[3] = gdJPos[1];
%	jpos_G2[4] = -gdJPos[0];
%
% ����� Configs - ��ʾ������5���ؽڽǵ�����
function [Flag, Configs] = IKine5DNew(robot)

%Q:��ô�涨������ŵģ�
%A:�����Գ�,Ӧ��base��1
l01 = robot.LinksLen(1);
l2 = robot.LinksLen(2);
l3 = robot.LinksLen(3);
l45 = robot.LinksLen(4);

CurConfigs = deg2rad(robot.CurJAs);
% ���������ڵ������1Ϊ����
Sol_Flag = ones(1,4);
Configs = NaN * ones(1,5);  % �������
theta = ones(4,5);     % 4�������ݴ�ռ�

% ��ͬ�гֶ˴������˸��ϵ㣬���ֲ����else����
if robot.FixedGripper == 1
    EETrMtx = robot.TarGraspMtx;
else
    Trx = [1 0 0 0;0 -1 0 0; 0 0 -1 0; 0 0 0 1];
    EETrMtx = Trx / robot.TarGraspMtx * Trx;
end

% �����theta1
% ���theta1
theta(1:2,1) = ClampJAngle(CurConfigs(1),atan2(EETrMtx(2,4),EETrMtx(1,4)));
% ��һ��
theta(3:4,1) = ClampJAngle(CurConfigs(1),atan2(-EETrMtx(2,4),-EETrMtx(1,4)));

% ���Լгַ����Ƿ��ڻ����˹���ƽ���ڣ����������жϣ�
vdot = dot(EETrMtx(1:3,3),[-EETrMtx(2,4),EETrMtx(1,4),0]);
% if abs(vdot) > 1e-12
%     EETrMtx(1:3,1:3) = modifyOriMtx(EETrMtx(1:3,4),EETrMtx(1:3,1));
% end

% �任����ؽ����
EETrMtx(1:3,4) = EETrMtx(1:3,4) - EETrMtx(1:3,3) * l45;

% ���theta5 �� theta3
for i = 1:2
    
    s1 = sin(theta(2*i-1,1));
    c1 = cos(theta(2*i-1,1));

    % ע�� si��ʾ�ӽǶ�ֱ�����������ֵ��si_��ʾ��λ�˾��������������ֵ
    s5_ = s1 * EETrMtx(1,1) - c1 * EETrMtx(2,1);
    c5_ = s1 * EETrMtx(1,2) - c1 * EETrMtx(2,2);
    
    % theta5
    theta(2*i-1:2*i,5) = ClampJAngle(CurConfigs(5),atan2(s5_,c5_));
    
    % theta3
    c3_ = ((EETrMtx(1,4)*c1 + EETrMtx(2,4)*s1)^2 + (EETrMtx(3,4) - l01)^2 - (l2^2+l3^2))/(2*l2*l3);
    s3_sq = 1 - c3_^2;
    
    if (abs(s3_sq) < 1e-12)   %������
        s3_ = 0; 
    elseif (s3_sq < 0)       %С���㣬�޽�
        Sol_Flag(2*i-1:2*i) = 0;
        continue;
    else
        s3_ = sqrt(s3_sq);
    end
    
    %/////����New�汾��
    theta(2*i-1,3) = ClampJAngle(CurConfigs(3),atan2(s3_,c3_));
    % ��һ�ֽ�,��ô������ֵ����ͬһ���ڴ�ռ䣡��
    theta(2*i,3) = ClampJAngle(CurConfigs(3),atan2(-s3_,c3_));
    
%     %/////����22�汾��
%         theta(2*i-1,3) = ClampJAngle(CurConfigs(3),atan2(s3_,c3_));
%      if theta(2*i-1,3) <-pi
%          theta(2*i-1,3) = theta(2*i-1,3) +2*pi;
%      elseif  theta(2*i-1,3) >pi
%          theta(2*i-1,3) = theta(2*i-1,3) -2*pi;
%      end
%     % ��һ�ֽ�,��ô������ֵ����ͬһ���ڴ�ռ䣡��
%     theta(2*i,3) = ClampJAngle(CurConfigs(3),atan2(-s3_,c3_));
%     if theta(2*i,3) <-pi
%          theta(2*i,3) = theta(2*i-1,3) +2*pi;
%      elseif  theta(2*i,3) >pi
%          theta(2*i,3) = theta(2*i-1,3) -2*pi;
%      end
end

if ~Sol_Flag(1) && ~Sol_Flag(3)   %���ؽ�ȷ���޽�
    Flag = 0;
    return;
end
    
% ���theta2 �� theta4
for i = 1:4
    
    if Sol_Flag(i)
        
        s1 = sin(theta(i,1));
        c1 = cos(theta(i,1));
        
        s3 = sin(theta(i,3));
        c3 = cos(theta(i,3));
        
        % theta2
        pzl01 =  EETrMtx(3,4) - l01;
        pxpy = EETrMtx(1,4) * c1 + EETrMtx(2,4) * s1;        
        s2_ = pzl01 * (l3 * c3 + l2) - pxpy * l3 * s3;  % ��ĸ�����Ϊ�����ɶ�������
        c2_ = pxpy * (l3 * c3 + l2) + pzl01 * l3 * s3;  % ��ĸ�����Ϊ�����ɶ�������
        
        theta(i,2) = ClampJAngle(CurConfigs(2),atan2(s2_,c2_));
        
        % theta4
        s234_ = EETrMtx(1,3) * c1 + EETrMtx(2,3) * s1;
        c234_ = - EETrMtx(3,3);
        
        theta234 = atan2(s234_,c234_);  %234֮��
        theta(i,4) = ClampJAngle(CurConfigs(4), theta234 - theta(i,2) - theta(i,3));
        
    end
end

% �ؽڽ���λ
JiontLimitUp = deg2rad([360,240,140,240,360]);
JiontLimitLow = deg2rad([-360,-60,-140,-60,-360]);

% ѡ����ѽ��
criterion = ones(1,4);      %������inf*ones(1,4)��һ����ֹ�������
result = 0;

for i = 1:4
    
    if Sol_Flag(i)       
        % �Ƿ�ΪNanԪ��
        if find(isnan(theta(i,:))==1,1)
            Sol_Flag(i) = 0;
            continue;
        end
    end
    if Sol_Flag(i)   % �н�ĲŽ��йؽڽǼ��
        % �Ƿ񳬳���λ
        if any(theta(i,:) > JiontLimitUp) || any(theta(i,:) < JiontLimitLow)      % ������λ
            Sol_Flag(i) = 0;
            continue; % ��ǰ������λ���
        end
    end
%%2020.3.16ע��byCH    
%     if Sol_Flag(i)
%         % �������������������ʵģ�
%         criterion(i) = abs(theta(i,1)-CurConfigs(1)) + ...
%             abs(theta(i,2)-CurConfigs(2)) + ...
%             abs(theta(i,3)-CurConfigs(3)) + ...
%             abs(theta(i,4)-CurConfigs(4));
%             %abs(theta(i,5)-CurConfigs(5));
%         if ~result  %��һ�εõ���Ч�⣬�Ƚ��г�ʼ��
%             result = i;
%             best = criterion(i);
%         elseif best > criterion(i)      %�ڶ����Լ��Ժ󣬶���Ч����бȽ�
%             result = i;                 %���ŵĻ����滻��ԭ�н�
%             best = criterion(i);
%         elseif abs(best - criterion(i))<1e-5
%             %2020.3.15 �޸ģ�ԭ��Ϊ��% if theta(i,2)>0   %ƫ������͹�Ĺ��ͣ����ڶ�����ת�ؽ������
%             if theta(i,3)<0        
%                  result = i;                
%             best = criterion(i);
%             else
%                continue; 
%             end
%         end
%     end
    if Sol_Flag(i)
        % �������������������ʵģ�
        criterion(i) = abs(theta(i,1)-CurConfigs(1)) + ...
            abs(theta(i,2)-CurConfigs(2)) + ...
            abs(theta(i,3)-CurConfigs(3)) + ...
            abs(theta(i,4)-CurConfigs(4));
            %abs(theta(i,5)-CurConfigs(5));
%          if ( theta(i,3) >= 0 ) && (theta(i, 2) >= 0 )  
         if theta(i,3)<=0  
            if ~result  %��һ�εõ���Ч�⣬�Ƚ��г�ʼ��
                result = i;
                best = criterion(i);
                      
            elseif best > criterion(i)      %�ڶ����Լ��Ժ󣬶���Ч����бȽ�
                result = i;                 %���ŵĻ����滻��ԭ�н�
                best = criterion(i);
            else
               continue; 
            end
        end
    end
    
end

if result
    Flag = 1;
    Configs(1,:) = rad2deg(theta(result,:));     %ѡ�����ŵĽ�������
else
    Flag = 0;
end

end


% ���ݵ�ǰ�Ƕ��Ż�����ת�ǣ��޶���[-360,+360]��
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

% �޸ļгַ���ʹ5DoFs Climbot�ܵ���
% Ҳ�������Ϊ�Ա�֤λ�ú͸�����̬�����n�У�Ϊ����
function OriMtx = modifyOriMtx(Pos,n)

% OriMtx = [n|o|a]

% ����гַ���
if abs(n(3)) > 1e-12
    alpha = atan2(Pos(2),Pos(1));
    tanA = tan(alpha);
    lambda = -(n(1)+n(2)*tanA)/n(3);
    a = n; % ��ʼ��
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
