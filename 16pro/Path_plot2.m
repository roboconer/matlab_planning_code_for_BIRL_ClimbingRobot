pointassignment;
p1=[0.3 4 3.5 1 0.3];p2=[0.2 0.1 4 3 0.2];p3=[0 0 0 0 0];
p4=[0.2 0.5 0.7 0.2];p5=[0.1 3.3 1.4 0.1];p6=[0.2 0.2 3 0.2];
p7=[0.8 1 3 2.7 0.8];p8=[0.3 1.7 1.3 0.5 0.3];p9=[2 3.8 4 2.90694 2];
plot3(p1,p2,p3,'--r','LineWidth',2)
hold on
plot3(p4,p5,p6,'--g','LineWidth',2)
plot3(p7,p8,p9,'--b','LineWidth',2)
plot3(px1',py1',pz1','ok','LineWidth',2)
plot3(px2',py2',pz2','or','LineWidth',2)
path1x=[3,1.16478,0.4441,0.57433];path1y=[2,1.73289,1.8004,1.5];path1z=[0,0,0.82759,2];
path2x=[3,1.24673,0.438261,0.57433];path2y=[2,1.80072,1.87651,1.5];path2z=[0,0,0.731028,2];
path3x=[3,1.16316,0.44252,0.63966,0.98832,3];path3y=[2 1.71542 1.78298 1.3066 0.9225 1.3];path3z=[0 0 0.82754 2.618 2.8349 4];
path4x=[3 1.26539 0.421386 0.599131 1.01175 3];path4y=[2 1.99985 2.07898 1.52519 0.767 1.3];path4z=[0 0 0.46618 2.16616 2.65091 4];
plot3(path3x,path3y,path3z,'-b*','LineWidth',3)
 plot3(path4x,path4y,path4z,'-ro','LineWidth',3)
 axis equal 
%plot3(path1x,path1y,path1z,'-ro','LineWidth',3)
%plot3(path2x,path2y,path2z,'-b*','LineWidth',3)