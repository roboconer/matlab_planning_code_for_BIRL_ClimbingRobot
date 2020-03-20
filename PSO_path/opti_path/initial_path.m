function [ particle_min,particle_max ] = initial_path( ps,pe,interval )
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   ������ͼ����,�ϰ���ķ��࣬��չ��������
[ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map;  
dis_pse=norm(ps-pe);       %��㵽�յ����
%interval=0.5;
particle_size=ceil(dis_pse/interval-1);            %uzhi����ά����������������
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
particle_min=zeros(1,particle_size);
particle_max=zeros(1,particle_size);
for i=1:particle_size                 %�����Ӹ�ά�ı仯��Χ����ת��Ϊ��ͼ����ϵ
    kpxy1(i,:)=(TR*[px1(i),0]'+ps')';
    kpxy2(i,:)=(TR*[px1(i),0.5]'+ps')';
end
for i=1:particle_size
    linese(:,:,i)=[kpxy1(i,:);kpxy2(i,:)];
end
for i=1:particle_size                     %����������ֱ�����ͼ�Ľ���
    [inter_p,sta] =linepolyintersection(linese(:,:,i),map_bor,1);
    if sta==2
        inter_l(2*i-1:2*i,:)=inter_p;
    end
end
for i=1:particle_size                          %ת��Ϊ��ֹ������ϵ
    rangey(2*i-1,:)=(TR1*inter_l(2*i-1,:)'+ ps1')';
    rangey(2*i,:)=(TR1*inter_l(2*i,:)'+ ps1')';
end
for i=1:particle_size
    particle_min(i)=min( rangey(2*i-1,2), rangey(2*i,2));    %���Ӹ�ά��Сֵ
    particle_max(i)=max( rangey(2*i-1,2), rangey(2*i,2));    %���Ӹ�ά���ֵ
end


end

