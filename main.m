clear;
%% 全局规划
%%全局落足点
x = [];

%%优化结果（无用）
fval = [];

%%全局路径生成（优化方法）
% opti_solution
% plot_solution(x);

%%粒子群方法
 [x,~] = APSO_pa();
 x = x(2:size(x,1)-1, :);

%% 壁面局部路径规划
%壁面路径规划落足点
wallFootStep_Cell = {};
%落足点序列
wallFootStep = [];

% 落足点类型
stepOption = [];
 
% 壁面路径规划
[wallFootStep_Cell, stepOption] = surfaces_astar(x);
wallFootStep = [];
for i = 1 : size(wallFootStep_Cell, 2)
    tmp = wallFootStep_Cell{i};
    wallFootStep = [wallFootStep; tmp];
end

%% 单步运动规划
% 除全局起始点和全局结束落足点的每个落足点
Ba = [];

% 每一步的基坐标系
Ba_R = [];

%生成的关节角离散序列
joint_ang = [];

%单步路径规划
 [ Ba,Ba_R,joint_ang ] =motionplan2(wallFootStep, stepOption);

%% G代码生成
jointangle_transform(joint_ang)
%% 仿真动画生成
motion_simulation(Ba, Ba_R, joint_ang, 'result.avi') 