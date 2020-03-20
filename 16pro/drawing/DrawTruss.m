%% ==========================================================
% 函数功能：绘制桁架
% 输入：truss - 要绘制的桁架
%       faceA - 透明度
function DrawTruss(truss,faceA)

    cpole_radii = 30;  % 圆柱杆的半径
    spole_dia = 40;     % 方杆截面对角线长
    
    for j =1:size(truss,1)
        if truss(j,7)
            drawcube(truss(j,1:3),truss(j,4:6),spole_dia,truss(j,8),'y',faceA);
        else
            drawcylinder(truss(j,1:3),truss(j,4:6),cpole_radii,'b',faceA,1);
        end
    end
    
end