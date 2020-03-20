clear all;%设置数据格式
format long;
%设置粒子群算法参数
max_iteration=200;         %最大迭代次数
swarm_size=100;             %种群规模（粒子数量）
ps= [1 0.5];               %起点位置
pe=[4.2 4.8];              %终点位置
dis_pse=norm(ps-pe);       %起点到终点距离
interval= 0.6;
particle_size=ceil(dis_pse/interval-1);            %粒子维数（求解变量个数）
%n_size=[68,234,160,5];            %每个过渡边界包含的线段数(每个壁面的过渡点数量）
px1=[interval:interval:interval*particle_size];
pxy=(pe-ps)/dis_pse;
TR=[pxy(1) -pxy(2);pxy(2) pxy(1)];
TR1=[pxy(1) pxy(2);-pxy(2) pxy(1)];        %旋转矩阵求逆
ps1=(-TR1*ps')';                           %平移向量求逆
kpxy1=zeros(particle_size,2);
kpxy2=zeros(particle_size,2);
linese=zeros(2,2,particle_size);
inter_p= zeros(2,2);
inter_l= zeros(2*particle_size,2);
rangey=zeros(2*particle_size,2);
sta =0;
particle_min=zeros(1,particle_size);
particle_max=zeros(1,particle_size);
for i=1:particle_size                 %求粒子各维的变化范围
    kpxy1(i,:)=(TR*[px1(i),0]'+ps')';
    kpxy2(i,:)=(TR*[px1(i),0.5]'+ps')';
end
for i=1:particle_size
    linese(:,:,i)=[kpxy1(i,:);kpxy2(i,:)];
end

[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map;               %导入地图并生成障碍区域

for i=1:particle_size                     %求粒子所在直线与地图的交点
    [inter_p,sta] =linepolyintersection(linese(:,:,i),map_bor,1);
    if sta==2
        inter_l(2*i-1:2*i,:)=inter_p;
    end
end
for i=1:particle_size
    rangey(2*i-1,:)=(TR1*inter_l(2*i-1,:)'+ ps1')';
    rangey(2*i,:)=(TR1*inter_l(2*i,:)'+ ps1')';
end
for i=1:particle_size
    particle_min(i)=min( rangey(2*i-1,2), rangey(2*i,2));    %粒子各维最小值
    particle_max(i)=max( rangey(2*i-1,2), rangey(2*i,2));    %粒子各维最大值
end
% particle_min=[0.001,0.001,0.001,0.001];   %粒子各维最小值
% particle_max=[1,1,1,1];     %粒子各维最大值
velocity_min=-1.5;             %粒子速度最小值
velocity_max=1.5;             %粒子速度最大值
w_max=0.9;                  %惯性权重最大值
w_min=0.4;                  %惯性权重最小值
%初始化粒子群算法变量
w=zeros(particle_size,1);                                             %线性递减惯性权重向量
c1=zeros(particle_size,1);                                            %学习因子C1向量
c2=zeros(particle_size,1);                                            %学习因子C2向量
order=zeros(swarm_size);                                            %粒子排序向量
fitness_size=particle_size+1;                                       %位置及适应度结合矩阵维数
particle=zeros(swarm_size,fitness_size,max_iteration+1);              %粒子群位置及适应度矩阵
velocity=zeros(swarm_size,particle_size,max_iteration+1);             %粒子群速度矩阵
pbest=zeros(swarm_size,fitness_size,max_iteration);                 %局部最优粒子位置及适应度矩阵
gbest=zeros(1,fitness_size,max_iteration);                          %全局最优粒子位置及适应度矩阵
num_sa=0;                                                           %记录最优值的重复度
%初始化粒子群
%初始化迭代数
z=1;
tic
%%%%%%%%初始化粒子群的位置，速度和适应度
for x=1:swarm_size
    for y=1:particle_size
        particle(x,y,z)=(particle_min(y)+(particle_max(y)-particle_min(y))*rand);
        velocity(x,y,(z+1))=-1.5+rand*3;
        %粒子位置更新函数UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
        particle(x,y,(z+1))=UpdateP1(particle,velocity,x,y,z,particle_min,particle_max,gbest);         
    end
    %适应度函数Fitness( particle,index,iter )
    particle(x,fitness_size,z)=Fitness1(particle,x,z,particle_size,interval,acr_obs,att_obs,byp_obs,TR,ps,pe);
end
%%%%%%%初始化局部最优粒子的位置和适应度
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
        particle(x,fitness_size,z)=Fitness1(particle,x,z,particle_size,interval,acr_obs,att_obs,byp_obs,TR,ps,pe); 
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
        c1(i)=2-z*1.9/max_iteration;%2*w(i);
        c2(i)=2-c1(i);%w_max-c1(i);
    end
     
if gbest(1,fitness_size,z-1)-gbest(1,fitness_size,z)<0.001
    num_sa=num_sa+1;
else
    num_sa=0;
end
%更新粒子
    for x=1:swarm_size
        for y=1:particle_size
            %粒子速度更新函数
%UpdateV( particle,velocity,pbest,gbest,index,dimen,iter,velocity_min,velocity_max,c1,c2 )
            velocity(x,y,(z+1))=UpdateV1(particle,velocity,pbest,gbest,x,y,z,velocity_min,velocity_max,c1,c2,w,num_sa);
            %粒子位置更新函数UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
            particle(x,y,(z+1))=UpdateP1(particle,velocity,x,y,z,particle_min,particle_max,gbest);         
        end
    end
end
toc
%%%%数据处理
DataManage1(particle,velocity,pbest,gbest,max_iteration,swarm_size,particle_size,fitness_size);