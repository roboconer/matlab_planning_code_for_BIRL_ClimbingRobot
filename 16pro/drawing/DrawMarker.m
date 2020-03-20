% =========================================
% ��������  �������壬��ʾ��ʼ��ĩ�˼гֵ�
% ���룺location - �������ĵ���ά����
%       color - ������ɫ
%       R - ����뾶
%       faceA - ͸����
% �������
function DrawMarker(location,color,R,faceA)

if nargin < 2
    color = 'c';   % ��ɫ
    R = 100;
    faceA = 0.65;
elseif nargin < 3
    R = 100;
    faceA = 0.65;
elseif nargin < 4
    faceA = 0.65;
end

% �������������
[x,y,z]=sphere(30);

% ������
surf(R*x+location(1),R*y+location(2),R*z+location(3),'FaceColor',color,'FaceAlpha',faceA,'LineStyle','none');

end