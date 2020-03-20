n=20;
for i=1:n+1
%     jerk1(i)=(acc1(i+1)-acc1(i))/td(i);
%     jerk2(i)=(acc2(i+1)-acc2(i))/td(i);
%     jerk3(i)=(acc3(i+1)-acc3(i))/td(i);
%     jerk4(i)=(acc4(i+1)-acc4(i))/td(i);
%     jerk5(i)=(acc5(i+1)-acc5(i))/td(i);
    if i==1
        t(i)=0;
    else
        t(i)=td(i-1)+t(i-1);
    end
end
for i=1:20
    if i==1||i==n
        time=linspace(t(i),t(i+1),45);
        acceleration1=(acc1(i+1)-acc1(i))/td(i)*(time-t(i))+acc1(i);
        acceleration2=(acc2(i+1)-acc2(i))/td(i)*(time-t(i))+acc2(i);
        acceleration3=(acc3(i+1)-acc3(i))/td(i)*(time-t(i))+acc3(i);
        acceleration4=(acc4(i+1)-acc4(i))/td(i)*(time-t(i))+acc4(i);
        acceleration5=(acc5(i+1)-acc5(i))/td(i)*(time-t(i))+acc5(i);
        velocity1=vel1(i)+acc1(i)*(time-t(i))+((acc1(i+1)-acc1(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity2=vel2(i)+acc2(i)*(time-t(i))+((acc2(i+1)-acc2(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity3=vel3(i)+acc3(i)*(time-t(i))+((acc3(i+1)-acc3(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity4=vel4(i)+acc4(i)*(time-t(i))+((acc4(i+1)-acc4(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity5=vel5(i)+acc5(i)*(time-t(i))+((acc5(i+1)-acc5(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
%         position4=Theta4(i)+vel4(i)*(time-t(i))+((acc4(i+1)-acc4(i))/td(i))*((time-t(i)).*(time-t(i)).*(time-t(i)))/6+acc4(i)*((time-t(i)).*(time-t(i)))/2;
%         plot(time,position4*180/3.14,'b*');xlabel('time(s)');ylabel('position (degree)');
        plot(time,velocity5,'-'); xlabel('时间(s)');ylabel('角速度 (rad/s)');
%           plot(time,acceleration5,'-'); xlabel('时间(s)');ylabel('角加速度 (rad/s^2)');
        hold on;
    else
    time=linspace(t(i),t(i+1),15);
     acceleration3=(acc3(i+1)-acc3(i))/td(i)*(time-t(i))+acc3(i);
     acceleration1=(acc1(i+1)-acc1(i))/td(i)*(time-t(i))+acc1(i);
        acceleration2=(acc2(i+1)-acc2(i))/td(i)*(time-t(i))+acc2(i);
         acceleration4=(acc4(i+1)-acc4(i))/td(i)*(time-t(i))+acc4(i);
        acceleration5=(acc5(i+1)-acc5(i))/td(i)*(time-t(i))+acc5(i);
        velocity3=vel3(i)+acc3(i)*(time-t(i))+((acc3(i+1)-acc3(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity1=vel1(i)+acc1(i)*(time-t(i))+((acc1(i+1)-acc1(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity2=vel2(i)+acc2(i)*(time-t(i))+((acc2(i+1)-acc2(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity4=vel4(i)+acc4(i)*(time-t(i))+((acc4(i+1)-acc4(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
        velocity5=vel5(i)+acc5(i)*(time-t(i))+((acc5(i+1)-acc5(i))/td(i))*((time-t(i)).*(time-t(i)))/2;
%         position4=Theta4(i)+vel4(i)*(time-t(i))+((acc4(i+1)-acc4(i))/td(i))*((time-t(i)).*(time-t(i)).*(time-t(i)))/6+acc4(i)*((time-t(i)).*(time-t(i)))/2;
%         plot(time,position4*180/3.14,'b*');xlabel('time(s)');ylabel('position (degree)');
         plot(time,velocity1,'-'); xlabel('时间(s)');ylabel('角速度 (rad/s)');
%          plot(time,acceleration5,'-'); xlabel('时间(s)');ylabel('角加速度 (rad/s^2)');
%       axis([0 40 -180 180]);
    hold on;
    end
end
axis([0 25 -0.6 0.6]);
