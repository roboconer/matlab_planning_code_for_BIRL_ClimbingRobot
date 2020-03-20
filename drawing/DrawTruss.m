%% ==========================================================
% �������ܣ��������
% ���룺truss - Ҫ���Ƶ����
%       faceA - ͸����
function DrawTruss(truss,faceA)

    cpole_radii = 30;  % Բ���˵İ뾶
    spole_dia = 40;     % ���˽���Խ��߳�
    
    for j =1:size(truss,1)
        if truss(j,7)
            drawcube(truss(j,1:3),truss(j,4:6),spole_dia,truss(j,8),'y',faceA);
        else
            drawcylinder(truss(j,1:3),truss(j,4:6),cpole_radii,'b',faceA,1);
        end
    end
    
end