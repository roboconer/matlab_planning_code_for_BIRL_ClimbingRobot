function new_point = rotation_x_minus90(point)
%point = point;
R = [1              0                   0;
     0              cos(-3.1415 / 2)     -sin(-3.1415 / 2);
     0              sin(-3.1415 / 2)     cos(-3.1415 / 2);];
new_point = (point * R);
end

% function new_point = rotation_x_minus90(point)
% point = point';
% R = [1              0                   0;
%      0              cos(3.1415 / 2)     -sin(3.1415 / 2);
%      0              sin(3.1415 / 2)     cos(3.1415 / 2);];
% new_point = (R*point)';
% end