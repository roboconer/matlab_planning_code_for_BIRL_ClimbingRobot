clear;
%%%%%%%%%%%%%%%全局规划
% 全局落足点
x = [];

% 优化结果（无用）
fval = [];

% 全局路径生成，
opti_solution;
plot_solution(x);

%%%%%%%%%%%%%%%壁面路径规划

% 壁面路径规划落足点
wallFootStep_Cell = {};
wallFootStep = [];

% 落足点类型
stepOption = [];

% 壁面路径规划
[wallFootStep_Cell, stepOption] = surfaces_astar(x);
for i = 1 : size(wallFootStep_Cell, 2)
    tmp = wallFootStep_Cell{i};
    wallFootStep = [wallFootStep; tmp];
end

%%%%%%%%%%%%%%%单步运动规划

% 
Ba = [];

%
Ba_R = [];

%生成的关节角
joint_ang = [];

%单步路径规划
[ Ba,Ba_R,joint_ang ] = motionplan2(wallFootStep, stepOption);
%motion_simulation(Ba, Ba_R, joint_ang, 'result.avi') 
