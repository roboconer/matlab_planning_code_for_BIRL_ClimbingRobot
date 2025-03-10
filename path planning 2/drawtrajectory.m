L1=280;L2=106;L3=375; 
i=24;
RT(:,:,1)=[cos(Theta1(i)) -sin(Theta1(i)) 0 0; sin(Theta1(i)) cos(Theta1(i)) 0 0; 0 0 1 (-L1-L2); 0 0 0 1];
RT(:,:,2)=[cos(Theta2(i)) -sin(Theta2(i)) 0 0; 0 0 1 0; -sin(Theta2(i)) -cos(Theta2(i)) 0 0; 0 0 0 1];
RT(:,:,3)=[cos(Theta3(i)) -sin(Theta3(i)) 0 (-L3); sin(Theta3(i)) cos(Theta3(i)) 0 0; 0 0 1 0;0 0 0 1];
RT(:,:,4)=[cos(Theta4(i)) -sin(Theta4(i)) 0 (-L3); sin(Theta4(i)) cos(Theta4(i)) 0 0; 0 0 1 0;0 0 0 1];
RT(:,:,5)=[cos(Theta5(i)) -sin(Theta5(i)) 0 0; 0 0 -1 0; sin(Theta5(i)) cos(Theta5(i)) 0 0; 0 0 0 1];
RT(:,:,6)=[1 0 0 0; 0 1 0 0; 0 0 1 ((L1+L2)); 0 0 0 1];
Tp6=RT(:,:,1)*RT(:,:,2)*RT(:,:,3)*RT(:,:,4)*RT(:,:,5)*RT(:,:,6);
Tp5=RT(:,:,1)*RT(:,:,2)*RT(:,:,3)*RT(:,:,4)*RT(:,:,5);
Tp4=RT(:,:,1)*RT(:,:,2)*RT(:,:,3)*RT(:,:,4);
Tp3=RT(:,:,1)*RT(:,:,2)*RT(:,:,3);
Tp2=RT(:,:,1)*RT(:,:,2);
Tp1=RT(:,:,1);
G1pos=[0;0;0];
I1pos=[Tp1(3,4);Tp1(2,4);Tp1(1,4)];
T1pos=[Tp2(3,4);Tp2(2,4);Tp2(1,4)];
T2pos=[Tp3(3,4);Tp3(2,4);Tp3(1,4)];
T3pos=[Tp4(3,4);Tp4(2,4);Tp4(1,4)];
I2pos=[Tp5(3,4);Tp5(2,4);Tp5(1,4)];
G2pos=[Tp6(3,4);Tp6(2,4);Tp6(1,4)];
line([G1pos(1),T1pos(1),T2pos(1),T3pos(1),G2pos(1)],[G1pos(2),T1pos(2),T2pos(2),T3pos(2),G2pos(2)],[G1pos(3),T1pos(3),T2pos(3),T3pos(3),G2pos(3)],'LineWidth', 1, 'Color',[1,0,0]);
