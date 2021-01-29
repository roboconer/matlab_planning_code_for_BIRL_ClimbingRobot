%clear all;
%�������ݸ�ʽ
function [P_path,path_l] = APSO_pa( )
%format long;
%��������Ⱥ�㷨����
[A,num_point,num_sur,Pstart,Pend] = surfaces;
num_s = num_sur-1;  
max_iteration=200;         %����������
swarm_size=100;             %��Ⱥ��ģ������������
particle_size=num_s;            %����ά����������������

tic
 [ C ] = chose_point(  );
           %ÿ�����ɱ߽�������߶���(ÿ������Ĺ��ɵ�������
n_size = zeros(1,num_s);
for i = 1:num_s
    n_size(i) = size(C{i},1);
end
%n_size=[68,234,160,5];            %ÿ�����ɱ߽�������߶���(ÿ������Ĺ��ɵ�������
particle_min=ones(1,num_s)*0.0005;   %���Ӹ�ά��Сֵ
particle_max=ones(1,num_s);     %���Ӹ�ά���ֵ
velocity_min=-0.5;             %�����ٶ���Сֵ
velocity_max=0.5;             %�����ٶ����ֵ
w_max=1;                  %����Ȩ�����ֵ
w_min=0.05;                  %����Ȩ����Сֵ
%��ʼ������Ⱥ�㷨����
w=zeros(max_iteration);                                             %���Եݼ�����Ȩ������
c1=zeros(max_iteration);                                            %ѧϰ����C1����
c2=zeros(max_iteration);                                            %ѧϰ����C2����
order=zeros(swarm_size);                                            %������������
fitness_size=particle_size+1;                                       %λ�ü���Ӧ�Ƚ�Ͼ���ά��
particle=zeros(swarm_size,fitness_size,max_iteration);              %����Ⱥλ�ü���Ӧ�Ⱦ���
velocity=zeros(swarm_size,particle_size,max_iteration);             %����Ⱥ�ٶȾ���
pbest=zeros(swarm_size,fitness_size,max_iteration);                 %�ֲ���������λ�ü���Ӧ�Ⱦ���
gbest=zeros(1,fitness_size,max_iteration);                          %ȫ����������λ�ü���Ӧ�Ⱦ���
 %pointassignment;
%��ʼ������Ⱥ
%��ʼ��������

z=1;
%%%%%%%%��ʼ������Ⱥ��λ�ã��ٶȺ���Ӧ��
for x=1:swarm_size
    for y=1:particle_size
        particle(x,y,z)=(particle_min(y)+(particle_max(y)-particle_min(y))*rand);
        velocity(x,y,(z+1))=-0.5+rand;
        %����λ�ø��º���UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
        particle(x,y,(z+1))=UpdateP(particle,velocity,x,y,z,particle_min,particle_max);         
    end
    %��Ӧ�Ⱥ���Fitness( particle,index,iter )
    particle(x,fitness_size,z)=Fitness(particle,x,z,n_size,C);
end
%%%%%%%%��ʼ���ֲ��������ӵ�λ�ú���Ӧ��
pbest(:,:,z)=particle(:,:,z);
%%%%%%%%��ʼ��ȫ���������ӵ�λ�ú���Ӧ��
gbest(1,:,z)=pbest(1,:,z);  
for x=1:swarm_size
    if pbest(x,fitness_size,z)<gbest(1,fitness_size,z)
       gbest(1,:,z)=pbest(x,:,z);    
    end
end
 
%%%%����
for z=2:max_iteration
%��������
    for x=1:swarm_size   
        %��Ӧ�Ⱥ���Fitness( particle,index,iter )  
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
    %���UpdateV��������
    %%��������
    for i=1:swarm_size
        for j=1:swarm_size
            if particle(i,fitness_size,z)>=particle(j,fitness_size,z)
                order(i)=order(i)+1; 
            end
        end
    end
    %%����Ȩ�ؼ���������
    for i=1:swarm_size
       % w(i)=w_min+(w_max-w_min)*(swarm_size-order(i))/(swarm_size-1);
       w(i)=w_max-z*(w_max-w_min)/max_iteration;
        c1(i)=2*w(i);
        c2(i)=c1(i);
    end
       
%��������
    for x=1:swarm_size
        for y=1:particle_size
            %�����ٶȸ��º���
%UpdateV( particle,velocity,pbest,gbest,index,dimen,iter,velocity_min,velocity_max,c1,c2 )
            velocity(x,y,(z+1))=UpdateV(particle,velocity,pbest,gbest,x,y,z,velocity_min,velocity_max,c1,c2,w);
            %����λ�ø��º���UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
            particle(x,y,(z+1))=UpdateP(particle,velocity,x,y,z,particle_min,particle_max);         
        end
    end
end
%%%%���ݴ���
[P_path,path_l]=DataManage(particle,velocity,pbest,gbest,max_iteration,swarm_size,particle_size,n_size,fitness_size,C);
toc
end