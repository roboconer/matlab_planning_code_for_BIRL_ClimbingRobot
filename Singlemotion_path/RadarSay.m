clc
clear

load ('data.mat')

Ba = [];

%
Ba_R = [];

%���ɵĹؽڽ�
joint_ang = [];

[ Ba,Ba_R,joint_ang] = motionplan2(wallFootStep,stepOption);
