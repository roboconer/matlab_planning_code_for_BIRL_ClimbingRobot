clc;clear;close all;  
%% ��ʼ����Ⱥ  
f= @(x)x .* sin(x) .* cos(2 * x) - 2 * x .* sin(3 * x); % ��������ʽ  
figure(1);ezplot(f,[0,0.01,20]);  
N = 50;                         % ��ʼ��Ⱥ����  
d = 1;                          % �ռ�ά��  
ger = 100;                      % ����������       
limit = [0, 20];                % ����λ�ò�������  
vlimit = [-1, 1];               % �����ٶ�����  
w = 0.8;                        % ����Ȩ��  
c1 = 0.5;                       % ����ѧϰ����  
c2 = 0.5;                       % Ⱥ��ѧϰ����   
for i = 1:d  
    x = limit(i, 1) + (limit(i, 2) - limit(i, 1)) * rand(N, d);%��ʼ��Ⱥ��λ��  
end  
v = rand(N, d);                  % ��ʼ��Ⱥ���ٶ�  
xm = x;                          % ÿ���������ʷ���λ��  
ym = zeros(1, d);                % ��Ⱥ����ʷ���λ��  
fxm = zeros(N, 1);               % ÿ���������ʷ�����Ӧ��  
fym = -inf;                      % ��Ⱥ��ʷ�����Ӧ��  
hold on  
plot(xm, f(xm), 'ro');title('��ʼ״̬ͼ');  
figure(2)  
%% Ⱥ�����  
iter = 1;  
record = zeros(ger, 1);          % ��¼��  
while iter <= ger  
     fx = f(x) ; % ���嵱ǰ��Ӧ��     
     for i = 1:N        
        if fxm(i) < fx(i)  
            fxm(i) = fx(i);     % ���¸�����ʷ�����Ӧ��  
            xm(i,:) = x(i,:);   % ���¸�����ʷ���λ��  
        end   
     end  
if fym < max(fxm)  
        [fym, nmax] = max(fxm);   % ����Ⱥ����ʷ�����Ӧ��  
        ym = xm(nmax, :);      % ����Ⱥ����ʷ���λ��  
 end  
    v = v * w + c1 * rand * (xm - x) + c2 * rand * (repmat(ym, N, 1) - x);% �ٶȸ���  
    % �߽��ٶȴ���  
    v(v > vlimit(2)) = vlimit(2);  
    v(v < vlimit(1)) = vlimit(1);  
    x = x + v;% λ�ø���  
    % �߽�λ�ô���  
    x(x > limit(2)) = limit(2);  
    x(x < limit(1)) = limit(1);  
    record(iter) = fym;%���ֵ��¼  
     x0 = 0 : 0.01 : 20;  
    plot(x0, f(x0), 'b-', x, f(x), 'ro');title('״̬λ�ñ仯')  
    pause(0.1)  
    iter = iter+1;  
end  
figure(3);plot(record);title('��������')  
x0 = 0 : 0.01 : 20;  
figure(4);plot(x0, f(x0), 'b-', x, f(x), 'ro');title('����״̬λ��')  
disp(['���ֵ��',num2str(fym)]);  
disp(['����ȡֵ��',num2str(ym)]);  