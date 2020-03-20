function [BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts()

%% ==========================================================
% 函数功能：读入所有模块的零部件（共8个）
% 输入:
%
% 输出： CADmodels

% filename = ['BandClamp';
%             'Gripper';
%             'HollowPart';
%             'IModule';
%             'TModuleP1';
%             'TModuleP2'];

%% 卡环
[F, V] = readCAD('BandClamp.stl');
V=[V';ones(1,numel(V)/3)];
V = tl(-50,50,-5.65)*rx(90)*V;
BandClamp = struct('name', 'BandClamp', 'faces', F, 'vertices', V(1:3,:)', 'facec', [1 0.8 0.2]);

%% 夹持器
[F, V] = readCAD('GModule.stl');
GModule = struct('name', 'Gripper', 'faces', F, 'vertices', V, 'facec', [0.5 0.5 0.5]);

%% 套筒 V3版长53.2mm
[F, V] = readCAD('HollowPart.stl');
% V=V';
% V = [V(1,:); V(2,:); V(3,:); ones(1,length(V))];
% V = tl(-50,-50,0)*V;
HollowPart = struct('name', 'HollowPart', 'faces', F, 'vertices', V, 'facec', [1 0.2 0.2]);

%% I模块
[F, V] = readCAD('IModule.stl');
IModule = struct('name', 'IModule', 'faces', F, 'vertices', V, 'facec', [1 0.2 0.2]);

%% T模块
[F, V] = readCAD('TModuleP1.stl');
TModuleP1 = struct('name', 'TModuleP1', 'faces', F, 'vertices', V, 'facec', [1 0.2 0.2]);

[F, V] = readCAD('TModuleP2.stl');
TModuleP2 = struct('name', 'TModuleP2', 'faces', F, 'vertices', V, 'facec', [1 0.2 0.2]);
%%  吸附模块
[F,V] = readCAD('SModule.stl');
SModule = struct('name', 'SModule', 'faces', F, 'vertices', V, 'facec', [0 0 0.7]);
end

% Homogeneous manipulation functions follow:
%
function Rx = rx(THETA)
% ROTATION ABOUT THE X-AXIS
%
% Rx = rx(THETA)
%
% This is the homogeneous transformation for
% rotation about the X-axis.
%
%	    NOTE:  The angle THETA must be in DEGREES.
%
THETA = THETA*pi/180;  % Note: THETA in radians.
c = cos(THETA);
s = sin(THETA);
Rx = [1 0 0 0; 0 c -s 0; 0 s c 0; 0 0 0 1];
end
%
function Ry = ry(THETA)
% ROTATION ABOUT THE Y-AXIS
%
% Ry = ry(THETA)
%
% This is the homogeneous transformation for
% rotation about the Y-axis.
%
%		NOTE: The angel THETA must be in DEGREES.
%
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Ry = [c 0 s 0; 0 1 0 0; -s 0 c 0; 0 0 0 1];
end
%
function Rz = rz(THETA)
% ROTATION ABOUT THE Z-AXIS
%
% Rz = rz(THETA)
%
% This is the homogeneous transformation for
% rotation about the Z-axis.
%
%		NOTE:  The angle THETA must be in DEGREES.
%
THETA = THETA*pi/180;  %Note: THETA is in radians.
c = cos(THETA);
s = sin(THETA);
Rz = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end
%
function T = tl(x,y,z)
% TRANSLATION ALONG THE X, Y, AND Z AXES
%
% T = tl(x,y,z)
%
% This is the homogeneous transformation for
% translation along the X, Y, and Z axes.
%
T = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
end