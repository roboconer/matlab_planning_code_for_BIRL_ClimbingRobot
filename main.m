clear;
%% ȫ�ֹ滮
%%ȫ�������
x = [];

%%�Ż���������ã�
fval = [];

%%ȫ��·�����ɣ��Ż�������
% opti_solution
% plot_solution(x);

%%����Ⱥ����
 [x,~] = APSO_pa();
 x = x(2:size(x,1)-1, :);

%% ����ֲ�·���滮
%����·���滮�����
wallFootStep_Cell = {};
%���������
wallFootStep = [];

% ���������
stepOption = [];
 
% ����·���滮
[wallFootStep_Cell, stepOption] = surfaces_astar(x);
wallFootStep = [];
for i = 1 : size(wallFootStep_Cell, 2)
    tmp = wallFootStep_Cell{i};
    wallFootStep = [wallFootStep; tmp];
end

%% �����˶��滮
% ��ȫ����ʼ���ȫ�ֽ���������ÿ�������
Ba = [];

% ÿһ���Ļ�����ϵ
Ba_R = [];

%���ɵĹؽڽ���ɢ����
joint_ang = [];

%����·���滮
 [ Ba,Ba_R,joint_ang ] =motionplan2(wallFootStep, stepOption);

%% G��������
jointangle_transform(joint_ang)
%% ���涯������
motion_simulation(Ba, Ba_R, joint_ang, 'result.avi') 