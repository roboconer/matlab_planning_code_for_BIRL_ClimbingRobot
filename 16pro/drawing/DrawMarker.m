% =========================================
% 函数功能  绘制球体，表示起始和末端夹持点
% 输入：location - 球体中心的三维坐标
%       color - 球体颜色
%       R - 球体半径
%       faceA - 透明度
% 输出：无
function DrawMarker(location,color,R,faceA)

if nargin < 2
    color = 'c';   % 青色
    R = 100;
    faceA = 0.65;
elseif nargin < 3
    R = 100;
    faceA = 0.65;
elseif nargin < 4
    faceA = 0.65;
end

% 生成球体的坐标
[x,y,z]=sphere(30);

% 画球体
surf(R*x+location(1),R*y+location(2),R*z+location(3),'FaceColor',color,'FaceAlpha',faceA,'LineStyle','none');

end