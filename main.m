clear;
%%%%%%%%%%%%%%%ȫ�ֹ滮
% ȫ�������
x = [];

% �Ż���������ã�
fval = [];

% ȫ��·�����ɣ�
opti_solution;
plot_solution(x);

%%%%%%%%%%%%%%%����·���滮

% ����·���滮�����
wallFootStep_Cell = {};
wallFootStep = [];

% ���������
stepOption = [];

% ����·���滮
[wallFootStep_Cell, stepOption] = surfaces_astar(x);
for i = 1 : size(wallFootStep_Cell, 2)
    tmp = wallFootStep_Cell{i};
    wallFootStep = [wallFootStep; tmp];
end

%%%%%%%%%%%%%%%�����˶��滮

% 
Ba = [];

%
Ba_R = [];

%���ɵĹؽڽ�
joint_ang = [];

%����·���滮
[ Ba,Ba_R,joint_ang ] = motionplan2(wallFootStep, stepOption);
%motion_simulation(Ba, Ba_R, joint_ang, 'result.avi') 
