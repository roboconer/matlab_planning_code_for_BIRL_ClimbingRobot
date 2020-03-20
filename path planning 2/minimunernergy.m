clear;
tic;
xg=200;yg=0;zg=-1100;xs=-720;ys=0;zs=-80; 
%xg=-400;yg=500;zg=-80;xs=-720;ys=0;zs=-80;
ps=[xs;ys;zs];pg=[xg;yg;zg];%起点和终点向量
Rs=[1 0 0;0 1 0; 0 0 1];           %起点和终点旋转矩阵
Rg=[0 0 1; 0 1 0; -1 0 0];
s=((xg-xs)^2+(yg-ys)^2+(zg-zs)^2)^0.5;      %起点到终点的距离
minenergy=10^40;num=0;                      %初始化能量值
z1lb=-s;
z1ub=s;
z2lb=-s;
z2ub=s;
alphalb=-10*pi/18;    %旋转关节上下限
alphaub=10*pi/18;
for m=1:2
    for i=1:9
        for j=1:9
            for k=1:19
                z1=z1lb+i*(z1ub-z1lb)/10;
                z2=z2lb+j*(z2ub-z2lb)/10;
                alpha=alphalb+k*(alphaub-alphalb)/20;
                [ThermalEnergy]=optimalpath(z1,z2,alpha);
                if ThermalEnergy<=minenergy-10^(-6)
                    minenergy=ThermalEnergy;
                    optz1=z1;optz2=z2;optalpha=alpha;
                end
            end
        end
    end
    z1lb=optz1-(z1ub-z1lb)/10;
    z1ub=optz1+(z1ub-z1lb)/10;
    z2lb=optz2-(z2ub-z2lb)/10;
    z2ub=optz2+(z2ub-z2lb)/10;
    alphalb=optalpha-(alphaub-alphalb)*pi/20;
    alphaub=optalpha+(alphaub-alphalb)*pi/20;
end
[acc1,acc2,acc3,acc4,acc5,vel1,vel2,vel3,vel4,vel5,Theta1,Theta2,Theta3,Theta4,Theta5,td,n,tor1,tor2,tor3,tor4,tor5]=outputacc(optz1,optz2,optalpha);
% for i=1:n+1
% %     jerk1(i)=(acc1(i+1)-acc1(i))/td(i);
% %     jerk2(i)=(acc2(i+1)-acc2(i))/td(i);
% %     jerk3(i)=(acc3(i+1)-acc3(i))/td(i);
% %     jerk4(i)=(acc4(i+1)-acc4(i))/td(i);
% %     jerk5(i)=(acc5(i+1)-acc5(i))/td(i);
%     if i==1
%         t(i)=0;
%     else
%         t(i)=td(i-1)+t(i-1);
%     end
% end
% for i=1:n
%     time=linspace(t(i),t(i+1),100);
%     acceleration1=(acc1(i+1)-acc1(i))/td(i)*(time-t(i))+acc1(i);
%     velocity1=vel1(i)+acc1(i)*(time-t(i))+((acc1(i+1)-acc1(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
%     position1=Theta1(i)+vel1(i)*(time-t(i))+((acc1(i+1)-acc1(i))/td(i))*((time-t(i)).*(time-t(i)).*(time-t(i)))/6+acc1(i)*((time-t(i)).*(time-t(i)))/2;
% %     plot(time,acceleration1);
%     hold on;
% end
toc;
%timer=toc;

    
    
    
    
    
    
    
    
    
    
                
            