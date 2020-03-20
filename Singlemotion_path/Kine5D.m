function [ T,joi_p ] =  Kine5D( joint_ang ,option)
%通过正运动学求机器人关节点的位姿,输入关节角，输出关节的位姿,根据option的大小来求解不同关节点相对于基座标的位姿
%global L1 L2;
joint_ang = deg2rad(joint_ang);
joi = joint_ang;

Len=[335,293.2,293.2,335];
%Len=[L1,L2,L2,L1]*1000;
T_1b = [cos(joi(1)) -sin(joi(1)) 0 0;sin(joi(1)) cos(joi(1)) 0 0;0 0 1 0;0 0 0 1];
T_21 = [cos(joi(2)) -sin(joi(2)) 0 0;0 0 -1 0;sin(joi(2)) cos(joi(2)) 0 Len(1);0 0 0 1];
T_32 = [cos(joi(3)) -sin(joi(3)) 0 Len(2);sin(joi(3)) cos(joi(3)) 0 0;0 0 1 0;0 0 0 1];
T_43 = [cos(joi(4)) -sin(joi(4)) 0 Len(3);sin(joi(4)) cos(joi(4)) 0 0; 0 0 1 0; 0 0 0 1];
T_54 = [cos(joi(5)) -sin(joi(5)) 0 0;0 0 -1 -Len(4);sin(joi(5)) cos(joi(5)) 0 0;0 0 0 1];
if option == 1%T_b_2
    T = T_1b*T_21;
    joi_p = T(1:3,4)';
elseif option == 2%T_b_3
    T = T_1b*T_21*T_32;
    joi_p = T(1:3,4)';
elseif option == 3%T_b_4
    T = T_1b*T_21*T_32*T_43;
    joi_p = T(1:3,4)';
elseif option == 4%T_b_5
    T = T_1b*T_21*T_32*T_43*T_54;
    joi_p = T(1:3,4)';
else
     h=msgbox('运动学求解失败','warn'); 
     uiwait(h,3);
     return;
end

end

