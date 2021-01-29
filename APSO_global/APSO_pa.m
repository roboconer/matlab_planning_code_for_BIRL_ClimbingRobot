%clear all;
%设置数据格式
function [P_path,path_l] = APSO_pa( )
%format long;
%设置粒子群算法参数
[A,num_point,num_sur,Pstart,Pend] = surfaces;
num_s = num_sur-1;  
max_iteration=200;         %最大迭代次数
swarm_size=100;             %种群规模（粒子数量）
particle_size=num_s;            %粒子维数（求解变量个数）

tic
 [ C ] = chose_point(  );
           %每个过渡边界包含的线段数(每个壁面的过渡点数量）
n_size = zeros(1,num_s);
for i = 1:num_s
    n_size(i) = size(C{i},1);
end
%n_size=[68,234,160,5];            %每个过渡边界包含的线段数(每个壁面的过渡点数量）
particle_min=ones(1,num_s)*0.0005;   %粒子各维最小值
particle_max=ones(1,num_s);     %粒子各维最大值
velocity_min=-0.5;             %粒子速度最小值
velocity_max=0.5;             %粒子速度最大值
w_max=1;                  %惯性权重最大值
w_min=0.05;                  %惯性权重最小值
%初始化粒子群算法变量
w=zeros(max_iteration);                                             %线性递减惯性权重向量
c1=zeros(max_iteration);                                            %学习因子C1向量
c2=zeros(max_iteration);                                            %学习因子C2向量
order=zeros(swarm_size);                                            %粒子排序向量
fitness_size=particle_size+1;                                       %位置及适应度结合矩阵维数
particle=zeros(swarm_size,fitness_size,max_iteration);              %粒子群位置及适应度矩阵
velocity=zeros(swarm_size,particle_size,max_iteration);             %粒子群速度矩阵
pbest=zeros(swarm_size,fitness_size,max_iteration);                 %局部最优粒子位置及适应度矩阵
gbest=zeros(1,fitness_size,max_iteration);                          %全局最优粒子位置及适应度矩阵
 %pointassignment;
%初始化粒子群
%初始化迭代数

z=1;
%%%%%%%%初始化粒子群的位置，速度和适应度
for x=1:swarm_size
    for y=1:particle_size
        particle(x,y,z)=(particle_min(y)+(particle_max(y)-particle_min(y))*rand);
        velocity(x,y,(z+1))=-0.5+rand;
        %粒子位置更新函数UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
        particle(x,y,(z+1))=UpdateP(particle,velocity,x,y,z,particle_min,particle_max);         
    end
    %适应度函数Fitness( particle,index,iter )
    particle(x,fitness_size,z)=Fitness(particle,x,z,n_size,C);
end
%%%%%%%%初始化局部最优粒子的位置和适应度
pbest(:,:,z)=particle(:,:,z);
%%%%%%%%初始化全局最优粒子的位置和适应度
gbest(1,:,z)=pbest(1,:,z);  
for x=1:swarm_size
    if pbest(x,fitness_size,z)<gbest(1,fitness_size,z)
       gbest(1,:,z)=pbest(x,:,z);    
    end
end
 
%%%%迭代
for z=2:max_iteration
%评价粒子
    for x=1:swarm_size   
        %适应度函数Fitness( particle,index,iter )  
        particle(x,fitness_size,z)=Fitness(particle,x,z,n_size,C); 
        if particle(x,fitness_size,z)<pbest(x,fitness_size,(z-1))
           pbest(x,:,z)=particle(x,:,z);
        else
           pbest(x,:,z)=pbest(x,:,(z-1));
        end   
    end
    for x=1:swarm_size
        if pbest(x,fitness_size,z)<gbest(1,fitness_size,(z-1))
           gbest(1,:,z)=pbest(x,:,z); 
        else
           gbest(1,:,z)=gbest(1,:,(z-1));
        end
    end
    %求解UpdateV函数参数
    %%粒子排序
    for i=1:swarm_size
        for j=1:swarm_size
            if particle(i,fitness_size,z)>=particle(j,fitness_size,z)
                order(i)=order(i)+1; 
            end
        end
    end
    %%计算权重及加速因子
    for i=1:swarm_size
       % w(i)=w_min+(w_max-w_min)*(swarm_size-order(i))/(swarm_size-1);
       w(i)=w_max-z*(w_max-w_min)/max_iteration;
        c1(i)=2*w(i);
        c2(i)=c1(i);
    end
       
%更新粒子
    for x=1:swarm_size
        for y=1:particle_size
            %粒子速度更新函数
%UpdateV( particle,velocity,pbest,gbest,index,dimen,iter,velocity_min,velocity_max,c1,c2 )
            velocity(x,y,(z+1))=UpdateV(particle,velocity,pbest,gbest,x,y,z,velocity_min,velocity_max,c1,c2,w);
            %粒子位置更新函数UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
            particle(x,y,(z+1))=UpdateP(particle,velocity,x,y,z,particle_min,particle_max);         
        end
    end
end
%%%%数据处理
[P_path,path_l]=DataManage(particle,velocity,pbest,gbest,max_iteration,swarm_size,particle_size,n_size,fitness_size,C);
toc
end