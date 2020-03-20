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
xx=linspace(0,24,100);
tor1=[tor1(1) tor1];
tor2=[tor2(1) tor2];
tor3=[tor3(1) tor3];
tor4=[tor4(1) tor4];
tor5=[tor5(1) tor5];
yyt1=interp1(t,tor1/10^9,xx,'spline');
yyt2=interp1(t,tor2/10^9,xx,'spline');
yyt3=interp1(t,tor3/10^9,xx,'spline');
yyt4=interp1(t,tor4/10^9,xx,'spline');
yyt5=interp1(t,tor5/10^9,xx,'spline');
plot(xx,yyt1,'b:');hold on;
plot(xx,yyt2,'g-.');
plot(xx,yyt3,'r-+','markersize',3);
plot(xx,yyt4,'c--');
plot(xx,yyt5,'m-');
legend('关节1','关节2','关节3','关节4','关节5');
xlabel('时间(s)');ylabel('力矩(Nm)');
axis([0 25 -60 60]);
