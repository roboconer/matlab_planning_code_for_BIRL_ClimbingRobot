% function drawpole
polepos=[0;0;0];
poleradius=20;
polelenth=2000;
bodyvc=[0 1.00 1];
bodyfc='flat';
a = 20;
theta = (0 : a-1)/a *2*pi;
x = poleradius*cos(theta);
y = poleradius*sin(theta);
z1 = polelenth/2*ones(1,a);
z2 = -z1;
vertexpole=[x x 0 0;y y 0 0;z1 -z1 polelenth/2 -polelenth/2];
for n = 1 : 3
    vertexpole(n,:) = vertexpole(n,:) + polepos(n);
end
faceup = [1:a; 2:a 1];
faceup(3: 4,:) = 2*a + 1;
facedown=[a+2:2*a a+1; a+1:2*a];
facedown(3: 4,:)=2*a +2;
faceside = [1:a; a+1:2*a; a+2:2*a a+1; 2:a 1];
face= [faceside faceup facedown];
patch('Vertices',vertexpole','Faces',face','FaceVertexCData',bodyvc,'FaceColor',bodyfc);axis equal;
view(3);grid on;
axis([-1200 800 -800 800 -1000 800]);



 polepos=[0;0;0];
 poleradius=20;
 polelenth=2400;
 bodyvc=[0 1.00 1];
 bodyfc='flat';
 a = 20;
 theta = (1 : a)/a *2*pi;
 x = poleradius*cos(theta)+400;
 y = poleradius*sin(theta);
 z1 = polelenth/2*ones(1,a);
 z2 = -z1;
 vertexpole=[z1 -z1 polelenth/2 -polelenth/2;y y 0 0;x x 400 400];
 for n = 1 : 3
     vertexpole(n,:) = vertexpole(n,:) + polepos(n);
 end
 faceup = [1:a; 2:a 1];
 faceup(3: 4,:) = 2*a + 1;
 facedown=[a+2:2*a a+1; a+1:2*a];
 facedown(3: 4,:)=2*a +2;
 faceside = [1:a; a+1:2*a; a+2:2*a a+1; 2:a 1];
 face= [faceside faceup facedown];
 patch('Vertices',vertexpole','Faces',face','FaceVertexCData',bodyvc,'FaceColor',bodyfc);axis equal;
 view(3);grid on;
 axis([-1200 500 -800 800 -1000 500]);