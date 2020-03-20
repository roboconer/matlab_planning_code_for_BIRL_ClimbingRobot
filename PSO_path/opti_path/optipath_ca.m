clear ;
[ ps,pe,interval ] = assignvalue;
[ particle_min,particle_max ] = initial_path( ps,pe,interval );
num_xsize=length(particle_min);
xinit=zeros(1,num_xsize);
options = optimset('MaxFunEvals',3000);

    %pinit=[2,1,3;2,1,3.3;3,1,4;3,1,4.3;4,1,2;4,1,2.3;3,2,2;3,2,2.3];
tic
[x,fval] = fmincon('min_path',xinit,[],[],[],[],particle_min,particle_max,'optimization_path',options)
toc