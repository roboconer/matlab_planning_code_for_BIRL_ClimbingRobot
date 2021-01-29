clc
clear

load ('data.mat')

Ba = [];

%
Ba_R = [];

%生成的关节角
joint_ang = [];

[ Ba,Ba_R,joint_ang] = motionplan2(wallFootStep,stepOption);
