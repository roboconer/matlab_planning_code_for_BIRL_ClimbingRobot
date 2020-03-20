clear all;%�������ݸ�ʽ
format long;
%��������Ⱥ�㷨����
max_iteration=200;         %����������
swarm_size=100;             %��Ⱥ��ģ������������
ps= [1 0.5];               %���λ��
pe=[4.2 4.8];              %�յ�λ��
dis_pse=norm(ps-pe);       %��㵽�յ����
interval= 0.6;
particle_size=ceil(dis_pse/interval-1);            %����ά����������������
%n_size=[68,234,160,5];            %ÿ�����ɱ߽�������߶���(ÿ������Ĺ��ɵ�������
px1=[interval:interval:interval*particle_size];
pxy=(pe-ps)/dis_pse;
TR=[pxy(1) -pxy(2);pxy(2) pxy(1)];
TR1=[pxy(1) pxy(2);-pxy(2) pxy(1)];        %��ת��������
ps1=(-TR1*ps')';                           %ƽ����������
kpxy1=zeros(particle_size,2);
kpxy2=zeros(particle_size,2);
linese=zeros(2,2,particle_size);
inter_p= zeros(2,2);
inter_l= zeros(2*particle_size,2);
rangey=zeros(2*particle_size,2);
sta =0;
particle_min=zeros(1,particle_size);
particle_max=zeros(1,particle_size);
for i=1:particle_size                 %�����Ӹ�ά�ı仯��Χ
    kpxy1(i,:)=(TR*[px1(i),0]'+ps')';
    kpxy2(i,:)=(TR*[px1(i),0.5]'+ps')';
end
for i=1:particle_size
    linese(:,:,i)=[kpxy1(i,:);kpxy2(i,:)];
end

[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map;               %�����ͼ�������ϰ�����

for i=1:particle_size                     %����������ֱ�����ͼ�Ľ���
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
    particle_min(i)=min( rangey(2*i-1,2), rangey(2*i,2));    %���Ӹ�ά��Сֵ
    particle_max(i)=max( rangey(2*i-1,2), rangey(2*i,2));    %���Ӹ�ά���ֵ
end
% particle_min=[0.001,0.001,0.001,0.001];   %���Ӹ�ά��Сֵ
% particle_max=[1,1,1,1];     %���Ӹ�ά���ֵ
velocity_min=-1.5;             %�����ٶ���Сֵ
velocity_max=1.5;             %�����ٶ����ֵ
w_max=0.9;                  %����Ȩ�����ֵ
w_min=0.4;                  %����Ȩ����Сֵ
%��ʼ������Ⱥ�㷨����
w=zeros(particle_size,1);                                             %���Եݼ�����Ȩ������
c1=zeros(particle_size,1);                                            %ѧϰ����C1����
c2=zeros(particle_size,1);                                            %ѧϰ����C2����
order=zeros(swarm_size);                                            %������������
fitness_size=particle_size+1;                                       %λ�ü���Ӧ�Ƚ�Ͼ���ά��
particle=zeros(swarm_size,fitness_size,max_iteration+1);              %����Ⱥλ�ü���Ӧ�Ⱦ���
velocity=zeros(swarm_size,particle_size,max_iteration+1);             %����Ⱥ�ٶȾ���
pbest=zeros(swarm_size,fitness_size,max_iteration);                 %�ֲ���������λ�ü���Ӧ�Ⱦ���
gbest=zeros(1,fitness_size,max_iteration);                          %ȫ����������λ�ü���Ӧ�Ⱦ���
num_sa=0;                                                           %��¼����ֵ���ظ���
%��ʼ������Ⱥ
%��ʼ��������
z=1;
tic
%%%%%%%%��ʼ������Ⱥ��λ�ã��ٶȺ���Ӧ��
for x=1:swarm_size
    for y=1:particle_size
        particle(x,y,z)=(particle_min(y)+(particle_max(y)-particle_min(y))*rand);
        velocity(x,y,(z+1))=-1.5+rand*3;
        %����λ�ø��º���UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
        particle(x,y,(z+1))=UpdateP1(particle,velocity,x,y,z,particle_min,particle_max,gbest);         
    end
    %��Ӧ�Ⱥ���Fitness( particle,index,iter )
    particle(x,fitness_size,z)=Fitness1(particle,x,z,particle_size,interval,acr_obs,att_obs,byp_obs,TR,ps,pe);
end
%%%%%%%��ʼ���ֲ��������ӵ�λ�ú���Ӧ��
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
        c1(i)=2-z*1.9/max_iteration;%2*w(i);
        c2(i)=2-c1(i);%w_max-c1(i);
    end
     
if gbest(1,fitness_size,z-1)-gbest(1,fitness_size,z)<0.001
    num_sa=num_sa+1;
else
    num_sa=0;
end
%��������
    for x=1:swarm_size
        for y=1:particle_size
            %�����ٶȸ��º���
%UpdateV( particle,velocity,pbest,gbest,index,dimen,iter,velocity_min,velocity_max,c1,c2 )
            velocity(x,y,(z+1))=UpdateV1(particle,velocity,pbest,gbest,x,y,z,velocity_min,velocity_max,c1,c2,w,num_sa);
            %����λ�ø��º���UpdateP( particle,velocity,index,dimen,iter,particle_min,particle_max )
            particle(x,y,(z+1))=UpdateP1(particle,velocity,x,y,z,particle_min,particle_max,gbest);         
        end
    end
end
toc
%%%%���ݴ���
DataManage1(particle,velocity,pbest,gbest,max_iteration,swarm_size,particle_size,fitness_size);