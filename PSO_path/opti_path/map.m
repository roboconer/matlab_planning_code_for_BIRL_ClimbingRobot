function [ map_bor,acr_obs,att_obs,byp_obs,hei_obs ] = map( map_border,obs_border,obs_height )
%MAP �˴���ʾ�йش˺�����ժҪ
%   ������ͼ����,�ϰ���ķ��࣬��չ��������
input_map;
r=0.2;
num_map = size(map_border,1);
map_bor = zeros(num_map,2);
map_bor = map_border;
num_obs = size(obs_border,1)/4;
obs_wid = zeros(num_obs,1);
n_acr = 0;n_att = 0;n_byp = 0;
for i=1:num_obs
    wid1=norm(obs_border((i-1)*4+2,:)-obs_border((i-1)*4+3,:));     %���ϰ�����
    wid2=norm(obs_border((i-1)*4+2,:)-obs_border((i-1)*4+1,:));
    obs_wid(i)=min(wid1,wid2);
end
for i=1:num_obs                                                    %���ϰ�����з���
    if obs_wid(i)>0.4 && obs_height(i)<0.428
        n_att=n_att+1;
        k_att(n_att)=i;
        continue;
    end
    if obs_wid(i)<0.167 && ((-0.8372)*obs_wid(i)+0.4809)>obs_height(i)
        n_acr=n_acr+1;
        k_acr(n_acr)=i;
        else 
        n_byp=n_byp+1;
        k_byp(n_byp)=i;    
    end
end
acr_obs=zeros(n_acr*4,4);
att_obs=zeros(n_att*4,6);
byp_obs=zeros(n_byp*4,4);
hei_obs=zeros(num_obs,1);
for i=1:n_byp
    hei_obs(i) = obs_height(k_byp(i));
end
for i =1:n_acr
    hei_obs(n_byp+i) = obs_height(k_acr(i));
end
for i=1:n_att
    hei_obs(n_byp+n_acr+i) = obs_height(k_att(i));
end
for i=1:n_acr         %��¼�ɿ�Խ�ϰ�������չ����
   acr_obs(4*i-3:4*i,1:2)= obs_border(4*k_acr(i)-3:4*k_acr(i),1:2);
   acr_obs(4*i-3,3:4)= acr_obs(4*i-3,1:2)+(acr_obs(4*i-3,1:2)-acr_obs(4*i,1:2))/norm(acr_obs(4*i-3,1:2)-acr_obs(4*i,1:2))*r + (acr_obs(4*i-3,1:2)-acr_obs(4*i-2,1:2))/norm(acr_obs(4*i-3,1:2)-acr_obs(4*i-2,1:2))*r;
   acr_obs(4*i,3:4)=acr_obs(4*i,1:2)+(acr_obs(4*i,1:2)-acr_obs(4*i-3,1:2))/norm(acr_obs(4*i,1:2)-acr_obs(4*i-3,1:2))*r + (acr_obs(4*i,1:2)-acr_obs(4*i-1,1:2))/norm(acr_obs(4*i,1:2)-acr_obs(4*i-1,1:2))*r;
   for j=1:2
     acr_obs(4*i-j,3:4)=acr_obs(4*i-j,1:2)+(acr_obs(4*i-j,1:2)-acr_obs(4*i-j-1,1:2))/norm(acr_obs(4*i-j,1:2)-acr_obs(4*i-j-1,1:2))*r + (acr_obs(4*i-j,1:2)-acr_obs(4*i-j+1,1:2))/norm(acr_obs(4*i-j,1:2)-acr_obs(4*i-j+1,1:2))*r;  
   end
end

for i=1:n_byp         %��¼�����ƿ����ϰ�������չ����
   byp_obs(4*i-3:4*i,1:2)= obs_border(4*k_byp(i)-3:4*k_byp(i),1:2);
   byp_obs(4*i-3,3:4)= byp_obs(4*i-3,1:2)+(byp_obs(4*i-3,1:2)-byp_obs(4*i,1:2))/norm(byp_obs(4*i-3,1:2)-byp_obs(4*i,1:2))*r + (byp_obs(4*i-3,1:2)-byp_obs(4*i-2,1:2))/norm(byp_obs(4*i-3,1:2)-byp_obs(4*i-2,1:2))*r;
   byp_obs(4*i,3:4)= byp_obs(4*i,1:2)+(byp_obs(4*i,1:2)-byp_obs(4*i-3,1:2))/norm(byp_obs(4*i,1:2)-byp_obs(4*i-3,1:2))*r + (byp_obs(4*i,1:2)-byp_obs(4*i-1,1:2))/norm(byp_obs(4*i,1:2)-byp_obs(4*i-1,1:2))*r;
   for j=1:2
     byp_obs(4*i-j,3:4)=byp_obs(4*i-j,1:2)+(byp_obs(4*i-j,1:2)-byp_obs(4*i-j-1,1:2))/norm(byp_obs(4*i-j,1:2)-byp_obs(4*i-j-1,1:2))*r + (byp_obs(4*i-j,1:2)-byp_obs(4*i-j+1,1:2))/norm(byp_obs(4*i-j,1:2)-byp_obs(4*i-j+1,1:2))*r;  
   end
end

for i=1:n_att         %��¼�����������ϰ�����������������
   att_obs(4*i-3:4*i,1:2)= obs_border(4*k_att(i)-3:4*k_att(i),1:2);
   att_obs(4*i-3,3:4)=att_obs(4*i-3,1:2)+(att_obs(4*i-3,1:2)-att_obs(4*i,1:2))/norm(att_obs(4*i-3,1:2)-att_obs(4*i,1:2))*r + (att_obs(4*i-3,1:2)-att_obs(4*i-2,1:2))/norm(att_obs(4*i-3,1:2)-att_obs(4*i-2,1:2))*r;  %��������
   att_obs(4*i,3:4)=att_obs(4*i,1:2)+(att_obs(4*i,1:2)-att_obs(4*i-3,1:2))/norm(att_obs(4*i,1:2)-att_obs(4*i-3,1:2))*r + (att_obs(4*i,1:2)-att_obs(4*i-1,1:2))/norm(att_obs(4*i,1:2)-att_obs(4*i-1,1:2))*r;
   for j=1:2
     att_obs(4*i-j,3:4)=att_obs(4*i-j,1:2)+(att_obs(4*i-j,1:2)-att_obs(4*i-j-1,1:2))/norm(att_obs(4*i-j,1:2)-att_obs(4*i-j-1,1:2))*r + (att_obs(4*i-j,1:2)-att_obs(4*i-j+1,1:2))/norm(att_obs(4*i-j,1:2)-att_obs(4*i-j+1,1:2))*r;  
   end
   att_obs(4*i-3,5:6)=att_obs(4*i-3,1:2)+(att_obs(4*i,1:2)-att_obs(4*i-3,1:2))/norm(att_obs(4*i,1:2)-att_obs(4*i-3,1:2))*r + (att_obs(4*i-2,1:2)-att_obs(4*i-3,1:2))/norm(att_obs(4*i-2,1:2)-att_obs(4*i-3,1:2))*r;  %��������
   att_obs(4*i,5:6)=att_obs(4*i,1:2)+(att_obs(4*i-3,1:2)-att_obs(4*i,1:2))/norm(att_obs(4*i-3,1:2)-att_obs(4*i,1:2))*r + (att_obs(4*i-1,1:2)-att_obs(4*i,1:2))/norm(att_obs(4*i-1,1:2)-att_obs(4*i,1:2))*r;
   for j=1:2
     att_obs(4*i-j,5:6)=att_obs(4*i-j,1:2)+(att_obs(4*i-j-1,1:2)-att_obs(4*i-j,1:2))/norm(att_obs(4*i-j-1,1:2)-att_obs(4*i-j,1:2))*r + (att_obs(4*i-j+1,1:2)-att_obs(4*i-j,1:2))/norm(att_obs(4*i-j+1,1:2)-att_obs(4*i-j,1:2))*r;  
   end
end
end

