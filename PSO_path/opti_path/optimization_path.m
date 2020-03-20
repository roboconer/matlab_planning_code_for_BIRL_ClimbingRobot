function [f,ceq] = optimization_path(x)
[ ps,pe,interval ] = assignvalue;
[ diswei ] = transform_map( x,ps,pe,interval );
num_si=length(x);
for i=1:num_si
    f(i)=diswei(i)-10;
end
ceq(1)=x(1)-0.01;
end