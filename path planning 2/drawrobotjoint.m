function drawrobotjoint
%clc;
%clear;
G1pos=[0;0;0];
I1pos=[-215;0;0];
T1pos=[-386;0;0];
T2pos=[-522;0;-349];
T3pos=[-466;0;-720];
I2pos=[-215;0;-720];
G2pos=[-80;0;-720];
jointradius=50; jointlenth=50;
BodyVC=[1.00 0.00 0.00];    %ºìÉ«   
BodyFC='flat';
 a = 20;
theta = (0 : a-1)/a *2*pi;
x = jointradius*cos(theta);
y = jointradius*sin(theta);
z1 = jointlenth/2*ones(1,a);
z2 = -z1;
vertexI1=[z1 -z1 jointlenth/2 -jointlenth/2;y y 0 0;x x 0 0];
vertexT1=[x x 0 0;z1 -z1 jointlenth/2 -jointlenth/2;y y 0 0];
vertexT2=[x x 0 0;z1 -z1 jointlenth/2 -jointlenth/2;y y 0 0];
vertexT3=[x x 0 0;z1 -z1 jointlenth/2 -jointlenth/2;y y 0 0];
vertexI2=[z1 -z1 jointlenth/2 -jointlenth/2;y y 0 0;x x 0 0];
for n = 1 : 3
    vertexI1(n,:) = vertexI1(n,:) + I1pos(n);
    vertexT1(n,:) = vertexT1(n,:) + T1pos(n);
    vertexT2(n,:) = vertexT2(n,:) + T2pos(n);
    vertexT3(n,:) = vertexT3(n,:) + T3pos(n);
    vertexI2(n,:) = vertexI2(n,:) + I2pos(n);
end
faceupI = [1:a; 2:a 1];
faceupI(3: 4,:) = 2*a + 1;
facedownI=[a+2:2*a a+1; a+1:2*a];
facedownI(3: 4,:)=2*a +2;
facesideI = [1:a; a+1:2*a; a+2:2*a a+1; 2:a 1];
face= [facesideI faceupI facedownI];
patch('Vertices',vertexI1','Faces',face','FaceVertexCData',BodyVC,'FaceColor',BodyFC);axis equal;
patch('Vertices',vertexT1','Faces',face','FaceVertexCData',BodyVC,'FaceColor',BodyFC);
patch('Vertices',vertexT2','Faces',face','FaceVertexCData',BodyVC,'FaceColor',BodyFC);
patch('Vertices',vertexT3','Faces',face','FaceVertexCData',BodyVC,'FaceColor',BodyFC);
patch('Vertices',vertexI2','Faces',face','FaceVertexCData',BodyVC,'FaceColor',BodyFC);view(3);
axis([-1200 500 -800 800 -1000 800]);
grid on;
 line([G1pos(1)-50,I1pos(1),T1pos(1),T2pos(1),T3pos(1),I2pos(1),G2pos(1)-50],[G1pos(2),I1pos(2),T1pos(2),T2pos(2),T2pos(2),I2pos(2),G2pos(2)],[G1pos(3),I1pos(3),T1pos(3),T2pos(3),T3pos(3),I2pos(3),G2pos(3)],'LineWidth', 6, 'Color',[1,0,0]);
 line([0,-50,-50,0],[-50,-50,50,50],[0,0,0,0],'LineWidth', 6, 'Color',[1,0,0]);
 line([0,-130,-130,0],[-50,-50,50,50],[-720,-720,-720,-720],'LineWidth', 4, 'Color',[1,0,0]);
