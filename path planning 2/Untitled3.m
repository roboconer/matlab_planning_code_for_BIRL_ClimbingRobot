x=2:5;
y=[2.44*10^4 2.12*10^4 2.039*10^4 2.036*10^4];
xx=linspace(2,5,1000);
yyt=interp1(x,y,xx,'spline');
plot(xx,yyt);
xlabel('参数个数');ylabel('能耗的描述量(N^2*m^2*s)');
set(gca,'XTick',2:5);
set(gca,'XTickLabel',{'2','3','4','5'});