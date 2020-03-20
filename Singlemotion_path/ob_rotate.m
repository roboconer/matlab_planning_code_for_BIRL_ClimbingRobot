function [ ob_pk ] = ob_rotate( ob_p,ang )
%将障碍物旋转一定角度并输出
%   此处显示详细说明
ang = deg2rad(ang);
T_ro = [cos(ang),-sin(ang);sin(ang),cos(ang)];
for i = 1: size(ob_p,1)
    ob_pk(i,:) = ob_p(i,:) - ob_p(1,:);
     ob_pk(i,:) = (T_ro*ob_pk(i,:)')';
     ob_pk(i,:) = ob_pk(i,:) + ob_p(1,:);
end

    
end

