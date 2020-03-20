function [ particle_min,particle_max ] = initial_path( ps,pe,interval )
%UNTITLED 此处显示有关此函数的摘要
%   环境地图生成,障碍物的分类，扩展区域生成
[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map;  
dis_pse=norm(ps-pe);       %起点到终点距离
%interval=0.5;
particle_size=ceil(dis_pse/interval-1);            %uzhi粒子维数（求解变量个数）
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
particle_min=zeros(1,particle_size);
particle_max=zeros(1,particle_size);
for i=1:particle_size                 %求粒子各维的变化范围，先转化为地图坐标系
    kpxy1(i,:)=(TR*[px1(i),0]'+ps')';
    kpxy2(i,:)=(TR*[px1(i),0.5]'+ps')';
end
for i=1:particle_size
    linese(:,:,i)=[kpxy1(i,:);kpxy2(i,:)];
end
for i=1:particle_size                     %求粒子所在直线与地图的交点
    [inter_p,sta] =linepolyintersection(linese(:,:,i),map_bor,1);
    if sta==2
        inter_l(2*i-1:2*i,:)=inter_p;
    end
end
for i=1:particle_size                          %转化为起止点坐标系
    rangey(2*i-1,:)=(TR1*inter_l(2*i-1,:)'+ ps1')';
    rangey(2*i,:)=(TR1*inter_l(2*i,:)'+ ps1')';
end
for i=1:particle_size
    particle_min(i)=min( rangey(2*i-1,2), rangey(2*i,2));    %粒子各维最小值
    particle_max(i)=max( rangey(2*i-1,2), rangey(2*i,2));    %粒子各维最大值
end


end

