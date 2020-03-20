clear
xinit=[0.2 0.2 0.1];
xzero=[0.1465 0 0.001];
xup=[0.293 0.2537 0.2537];

   options = optimset('MaxFunEvals',2000);
    %pinit=[2,1,3;2,1,3.3;3,1,4;3,1,4.3;4,1,2;4,1,2.3;3,2,2;3,2,2.3];
tic
[x,fval] = fmincon('max_height',xinit,[],[],[],[],xzero,xup,'opti_height',options)   %优化求解函数
toc