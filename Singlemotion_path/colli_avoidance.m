function [lastbas_R,laststp_R,lastjoi,joint_ang ] = colli_avoidance( M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option)

%  �����浥�������˶�
%[ joint_ang,p_s ] = joint_angle(stp,enp,bap);
global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;
[BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();
CurHMatrix  = eye(4);
enp_o = enp;
stp_o = stp;
bap_o = bap;
bap1 = bap;
wrobot = struct('DoFs',5,'LinksLen',[335,293.2,293.2,335],'ModuleRadii',50,...
'CurGraspMtx',CurHMatrix,'CurJAs',[0 8.4728 -16.9456 8.4728 0],'FixedGripper',1,'TarGraspMtx',eye(4,4),...
'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
 wrobot.CurJAs = fir_joi;
dis_warn = 0.04;  %ע�����Ϊ4cm
dis_safe = 0.02;  %��С��ȫ����Ϊ2cm
dis = 10000;
TO_SURFACE_DISTANCE = 0.05;%�����滮ʱ������̧��ľ���
vec_lin = zeros(1,3);
vec_eff = zeros(1,3);
num_ob = floor(size(obs_p,1)/8);
wrobot.CurGraspMtx(1:3,4) = bap*1000;
wrobot.CurGraspMtx(1:3,1:3) = bas_R;
figure (3);
  pp = patch(M_sur(:,1)*1000,M_sur(:,2)*1000,M_sur(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.5,'LineWidth',1);%�����ͼ
  %��ͼ��Χ���� by:CH
  set(gca,'xlim',[-100,1500])%��x���ϵ�ȡֵ��Χ����Ϊ[-100,1500] 
  xlabel('x');
  set(gca,'ylim',[-1500,1500])%��y���ϵ�ȡֵ��Χ����Ϊ[-1500,1500] 
  ylabel('y');
  set(gca,'zlim',[200,1500])%��z���ϵ�ȡֵ��Χ����Ϊ[200,1500] 
  zlabel('z');
  hold on 
   
   if num_ob>=1          %�ж��ϰ����Ƿ����
      obp_tran(obs_p);      %�ϰ����ͼ
     for i = 1:num_ob
              obs_p2(i*8-7:i*8,:) = obs_p(i*8-7:i*8,:);
     end
   end 

plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);%�����
plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);%���յ�
axis equal

num_mp = size(M_sur,1);
Vnm=cross(M_sur(2,:)-M_sur(1,:),M_sur(3,:)-M_sur(2,:));   %�������η�����
Vn_norm=norm(Vnm);
Vnm=Vnm/Vn_norm;
bas_x = (M_sur(2,:)-M_sur(1,:))/norm(M_sur(2,:)-M_sur(1,:));   %��������˶��Ļ�����ϵ
bas_z = Vnm(1,:); 
bas_y = cross(bas_z,bas_x);
% bas_R = [bas_x',bas_y',bas_z'];
 wor_to_bas_R = bas_R';   %��������ϵ�ڻ�����ϵ�ı�ʾ
 wtoba_p = (-wor_to_bas_R*bap')';
 Vn = (wor_to_bas_R*Vnm')';
 
for j = 1:num_mp
    m_sur(j,:) = (wor_to_bas_R*M_sur(j,:)')' + wtoba_p; 
end

 enp = (wor_to_bas_R*enp')'+ wtoba_p;   %Ŀ����ڻ�����ϵ�µı�ʾ
 stp = (wor_to_bas_R*stp')'+ wtoba_p;   %��ʼ������ڻ������µı�ʾ
 enp = enp + Vn*TO_SURFACE_DISTANCE;    %Ŀ��������ĸ���·������
 if num_ob>=1                            %�ж��ϰ����Ƿ���ڣ������ڣ�ת������������ϵ��
 for i = 1:size(obs_p2,1)
     obs_p2(i,:) = (wor_to_bas_R*obs_p(i,:)')'+ wtoba_p;
 end
 %obs_p2 = obs_p;
 end
%      wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
%      wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %����ĩ������ϵ����  

%//////////////////////////////////////////���ʼ��Ŀ��ؽڽ�
wrobot.TarGraspMtx(1:3,1:3) = stp_R;
wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
[Flag,sta_ang] = IKine5DNew(wrobot);                %���������ʼ���͵Ĺؽڽ�
un_sta = sta_ang(1,5);                               %�����˲������ת�ؽڣ��ǻ�����ֵ  
wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %�������Ŀ�깹�͵Ĺؽڽ�
wrobot.CurJAs = sta_ang;
[Flag, end_ang] = IKine5DNew(wrobot); 
% %***********************2020.3.15 �޸�:�����Ŀ��ؽڱ����������ؽڵ��ж�
% if end_ang(1,3) > 0
%     end_ang(1,3) = -end_ang(1,3);
%     end_ang(1,2) = -end_ang(1,2);
%     end_ang(1,4) = -end_ang(1,4);
% end
% %//////////////////////////////////////////end

%/////////////////////%I�ؽڻ�һ��������ת
if so_option == 2
    diff_eng =360-abs(end_ang(1) - sta_ang(1));
    if end_ang(1) - sta_ang(1)>0
        end_ang(1) = sta_ang(1) - diff_eng;
    else
         end_ang(1) = sta_ang(1) + diff_eng;
    end     
       Flag
end
%*********************end

wrobot.TarJAs =sta_ang;
 % DrawRobotmo(wrobot,1);
wrobot.TarJAs =end_ang;
%  DrawRobotmo(wrobot,1);
  end_ang
%%�����ʼ��Ŀ�����ͬһ���������
%//////////////////////////////// 1�����û���������5cm,����ʼ���Ŀ��������һ�ξ�������ƽ���������� 
%2020.2.27�ģ������������������ĳ�ȫ�֣������޸�
 for i = 1:TO_SURFACE_DISTANCE * 100             
    wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
    if i > 1 
        wrobot.CurJAs = joint_ang(i-1,:);
    end
    %@@@@@@@@@@@@@@@@@@@@@@@IKine5DNew���ص��ǲ��ǽǶ����� answer�����ǣ����ص���Ŀ��Ƕ�
    [Flag, joint_ang(i,:)] = IKine5DNew(wrobot);%Flag�����˶�ѧ�Ƿ��н�ı�־��joint_ang(i,:)�Ǹ����ؽڵĽǶ�
 end
 stp = stp + Vn * TO_SURFACE_DISTANCE;   
 bap =  Vn * TO_SURFACE_DISTANCE;
%//////////////////////////////// end


num_ang = 5;
i = 0;
%///////////////////////////////////////////��һ��while
while norm(joint_ang(num_ang,:)-end_ang)>1e-4%�������5cm�����һ����Ŀ����̬�㹻���ƵĻ��Ͳ��������ѭ��
    
      [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);%joint_ang(num_ang,:)�����һ֡�ĽǶȣ������joi_pp�ǻ�������ϵ�µ�
      joi_pp = (bas_R*joi_pp')'+ bap1*1000;%joi_ppת����������ϵ��
      plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);%�ۺ�ɫԲȦ����������
      joi_diffe = abs(end_ang - joint_ang(num_ang,:));   %ÿ���ؽڽ�֮��Ĳ�ֵ�㹻С��ֱ�ӵ���Ŀ���
    
    %////////////���˼г�����ÿ���ǶȲ�С��3��Ļ�
    if joi_diffe(1:4)< 3
    num_ang = num_ang +1;
    joint_ang(num_ang,:) = end_ang;
    break;
    end
    %*****************************end
      
    %////////////��������涨����,��step_type����2���
    if num_ang>300
    if so_option == 1%����Ǳ���滮
       %   break;
          %@@@@@@@@@@@@@@@@Ϊʲô��������涨������Ҫ��step_type����2��
          [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance( ...
              M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
          return;
    else
         h=msgbox('�滮�������޽��','warn'); 
         uiwait(h,2);
          break;
    end
    end
    %*****************************end
     
     %////////////����ϰ��ﲻ���ڣ������ǶȲ�ı���������һ���Ƕ�����
    if num_ob <1    
      [max_diffe,num_j]=max(abs(end_ang-joint_ang(num_ang,:)));
      num_inter = floor(max_diffe/3);
      joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
      num_ang = num_ang + 1; 
      continue;
    end
    %*****************************end
    
      dis1 = ones(1,num_ob)*10000;
      dis2 = ones(1,num_ob)*10000;
      
    %////////////%���������������ʧЧʱ���ã��þ���������޸ģ���ĩ�˵�xy��ȥĿ��xy��
    for j = 1: 4   %���ĸ��ؽڵ�
        [ T,joi_p(j,:)] =  Kine5D(joint_ang(num_ang,:) ,j);
    end 
    joi_p = joi_p/1000;
    %���ﲻ��
    if  norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0])<0.08           
       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
       num_inter = floor(max_diffe/3);
       joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
       num_ang = num_ang + 1; 
        continue;
    end
    %*****************************end
    
   %////////////%ĩ�˾��������������step_type����2���
   %if norm(joi_p(4,:))>0.586 || norm(joi_p(4,:))<0.38  %���������˹����ռ䷶Χ
    if norm(joi_p(4,:))<0.38  %ĩ�˾����������
        if so_option == 1
          [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
          return;
    else
          h=msgbox('ĩ�˹������ؽڽ����ʧ��','warn'); 
          uiwait(h,2);
          break;
        end
    end
    %*****************************end

    %////////////%�������ϰ������·ֱ�����������ϰ��ľ����Լ�������ϰ��ľ��룬����������ȡ��С��dis��
    for k = 1:num_ob
            if insidepoly3(obs_p2(8*k-7,:),m_sur) ==1%�����ǰ�ϰ������������
                 eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);%��ȡ����Բ��ƽ��
       % [ dis1(k),po1,po2] = dis_compute(joi_p(4,:),obs_p2(k*8-7:k*8,:),1);   %�û�����ĩ�˵�ȥ�Ա����ϰ�������
      [ dis1(k),po1,po2] = dis_compute(eff_p,obs_p2(k*8-7:k*8,:),6); 
             else
                continue;
            end    
    end
    %�������ϰ���
    for k = 1:num_ob
        if insidepoly3(obs_p2(8*k-7,:),m_sur) == 0
             for j=1:3            %������˱������˾����ϰ������С����
                 [ dist(j),po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p2(i*8-7:i*8,:),2);
                   dis2(k) = min(dist);
             end
        else
            continue;
        end
    end
     dis2 = dis2 - TO_SURFACE_DISTANCE;
     [mind,min_ob2] =min(dis2);
     [mind,min_ob1] =min(dis1);
    dis = min([min(dis1),min(dis2)]);
    %*****************************end
    
    %////////////%������������Ҫע��ľ��룬���������Խ��йؽڽǶȹ滮
    if dis >=dis_warn              
       [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
       num_inter = floor(max_diffe/3);
       joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        num_ang = num_ang + 1;
        continue;
    end
    %*****************************end
    
    %////////////%�ѷ�����ײ����step_type����2 @@@@@@@@@@@@@@@@@@@@@ΪʲôҪ����2
    if dis <= 0.003       
        if so_option == 1
              [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance( M_sur,stp_o,enp_o,bap_o,obs_p,bas_R,stp_R,fir_joi,2);
              return;
        else
              h=msgbox('������ײ���滮ʧ��','warn'); 
              uiwait(h,2);
              break;
        end
    end
   %*****************************end
   
   %////////////%���루���̸������ϰ�����ڰ�ȫ���뵫����Ҫע��ľ����� 
   %2020.2.27��main��ʱ��û�н�������if
   %��һ��ʦ������ʲô��˼
   if dis <dis_warn && dis>dis_safe && min(dis1)<min(dis2) 
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + ...
            [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
        %����i�ؽ���ת3��
        if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 ||...
                abs(end_ang(1)-joint_ang(num_ang+1,1))<=2
            
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
             num_inter = floor(max_diffe/1.5);   %�ò����������޸�
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        elseif  joi_p(4,3)-obs_p2(min_ob1*8-3,3)<0.02 %Ϊʲô�� obs_p2(min_ob1*8-3,3) 
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 3 0 -3 0];
        end
              num_ang = num_ang +1;
              continue;
   end
   %*****************************end
   
    %////////////%����С�ڰ�ȫ����,�ж��ϰ������������½��Ƿ�������ϰ���������С�����Դ˵������˶�����
   if dis <= dis_safe  && min(dis1)<min(dis2)   %���ĩ�����ϰ������
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 3 0 -2 0];%�������һ��
        [ dis1kt,po1,po2] = dis_compute(joi_p(4,:),obs_p2(min_ob1*8-7:min_ob1*8,:),1);%�û�����ĩ�˵�ȥ������ϰ�������
        pok = (po1-po2)/norm(po1-po2);
        joi_ps = point_pro_sur(joi_p(4,:),M_sur);
        
         %////////////%�ж��ϰ����ĩ�˵��Ӱ������˶��ķ����Ƿ���ͬ���������ڱ����ͶӰ��ͻ���֮�������Ϊ�ֽ磬�ж��ϰ������յ��Ƿ�λ��ͬһ�ࣩ
         if dot(cross(joi_ps-bap,po1 - po2),cross(joi_ps-bap,enp-Vn*TO_SURFACE_DISTANCE-bap))>0   
             joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                 [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*(1+(dis_safe-dis)*60),0,2,0,0];%���λ��ͬһ��
         else
             joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                 [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-(1+(dis_safe-dis)*60),0,2,0,0];%�����λ��ͬһ��
         end
         %*****************************end
         
         %////////////%�������˶�����ƫ����� @@@@@@@@@Ϊʲô������������
         if (end_ang(1)-joint_ang(num_ang-2,1))*(end_ang(1)-joint_ang(num_ang-3,1))<0 
             if  norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0])>=0.1    
                 vec_re =( [enp(1),enp(2),0]-[joi_p(4,1),joi_p(4,2),0])/norm([joi_p(4,1),joi_p(4,2),0]-[enp(1),enp(2),0]);
                 next_p = joi_p(4,:) + vec_re * 0.01;
                 wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
                 wrobot.TarGraspMtx(1:3,1:3) = T(1:3,1:3);
                 wrobot.CurJAs=joint_ang(num_ang,:);
                 [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(wrobot);
             if  Flag == 0
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [0 -1 2 -1 0];
             end
             else
                   [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
                   num_inter = floor(max_diffe/2);
                   joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
             end
             num_ang = num_ang + 1;
             continue;
         end
         %*****************************end
         
          %////////////%@@@@@@@@@���￴�����ڸ���
         joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
             [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*2,0,0,0,0];
         for j = 1: 4   %���ĸ��ؽڵ�
            [ T,joi_p2(j,:)] =  Kine5D(joint_ang(num_ang+1,:) ,j);
         end 
         joi_p2 = joi_p2/1000;
         eff_p2 = cir_seem_poly( joi_p2(4,:),joi_p2(4,:)-joi_p2(3,:),0.2,20);
           %�û�����ĩ�˵�ȥ�Ա����ϰ�������     
         [ dis1kt,po1,po2] = dis_compute(eff_p2,obs_p2(min_ob1*8-7:min_ob1*8,:),6);
%          poo = [po1;po2];
%          plot3(poo(:,1),poo(:,2),poo(:,3),'-r');
         vec_f = dot(pok,Vn);
         if abs(vec_f)<0.01 && dis1kt <dis
            joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-5,0,0,0,0];
            num_ang = num_ang + 1;
            continue;
         end
         if dis1kt <dis  || num_ang>10 
             joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:) + ...
                 [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*-2,0,0,0,0];
             num_ang = num_ang + 1;
             continue;
         end      
         num_ang = num_ang + 1;
         continue;
   end
   %*****************************end
   
        %%�ռ��ϰ��ﲿ��
   if dis <dis_warn && dis>dis_safe && min(dis2)<min(dis1) %������ڰ�ȫ���뵫����Ҫע��ľ�����
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
        %/////////////////@@@@@@@@@@@Ϊʲô������ж����ݣ���
        if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 || abs(end_ang(1)-joint_ang(num_ang+1,1))<=2 %����I�ؽ��˶���goal_angle+-3�㷶Χ
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
             num_inter = floor(max_diffe/3);
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        end
       num_ang = num_ang +1; 
       continue;
   end
   
     %////////////%����С�ڰ�ȫ���루���������
     if dis <= dis_safe  && min(dis2)<min(dis1)   
      [vec_repul] = repul_veccompute(obs_p2(min_ob2*8-7:min_ob2*8,:),m_sur,joi_p);%�ϰ���Ի����������Ӱ��
      effector_p = point_pro_sur(joi_p(4,:),m_sur);
      vec_eff = effector_p/norm(effector_p);
        if dot(vec_repul,Vn)<-0.7 % ����135��%@@@@@@@@@@@@@@@@ΪʲôҪ�����жϣ���
            if joint_ang(num_ang,2)<-30 || joint_ang(num_ang,4) < -30
                 joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 -0.5 -1 3 0];
            else
                 joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 -1 2 -1 0];
            end
                       
        elseif dot(vec_repul,vec_eff)<-0.7 % ����135��%@@@@@@@@@@@@@@@@ΪʲôҪ�����жϣ���
            joint_ang(num_ang+1,:) = joint_ang(num_ang,:)+[0 1 -2 1 0];    
        end
         if (end_ang(1)-joint_ang(num_ang+1,1))*(end_ang(1)-joint_ang(num_ang,1))<0 ||...
                 abs(end_ang(1)-joint_ang(num_ang+1,1))<=3
            [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
             num_inter = floor(max_diffe/3);
             joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        end
       num_ang = num_ang + 1;
       continue;
     end
     %*****************************end
     
end
 %////////////%ʹ������ĩ�˵���Z�᷽�򿿽�TO_SURFACE_DISTANCE * 100cm
 for i = 1:TO_SURFACE_DISTANCE * 100              
      [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
      end_p = end_p - Vn*(0.01)*1000;
      wrobot.TarGraspMtx(1:3,1:3) = T(1:3,1:3);
%        T(1:3,2) = T(1:3,2)*-1;
%        T(1:3,3) = T(1:3,3)*-1; 
      wrobot.TarGraspMtx(1:3,4) = end_p';
      wrobot.CurJAs=joint_ang(num_ang+i-1,:);
       [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(wrobot);
       if Flag == 0 
            h=msgbox('�½������޽�','warn'); 
        uiwait(h,2);
        break;
       end
 end
num_ang = num_ang + 6;
joint_ang = [sta_ang;joint_ang];
num_joi = size(joint_ang,1);
joint_ang(:,5) = un_sta;
 %*****************************end
 

 
  wrobot.TarJAs =joint_ang(6,:);
  DrawRobotmo(wrobot,1);%����㴦������̧���ģ��
   wrobot.TarJAs =joint_ang(num_ang-6,:);
   DrawRobotmo(wrobot,1);%���յ㴦������̧���ģ��
   aa = abs(floor((end_ang(1)-sta_ang(1))/4));

num_pro = num_joi - 10;                  %%��·����ƽ������
 num_pro = floor(num_pro/2);
 for i = 1: num_pro
    dis1 = ones(1,num_ob)*10000;
    dis2 = ones(1,num_ob)*10000;
    joint_ang2 = (joint_ang(i*2-1+5,:)+joint_ang(i*2+1+5,:))/2;   
    if norm(joint_ang(i*2+5,:)-joint_ang2)<0.01
        continue;
    end
    for j = 1: 4   %���ĸ��ؽڵ�
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
    end 
    
    joi_p = joi_p/1000;
    
 for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==1
        eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);
   % [ dis1(k),po1,po2] = dis_compute(joi_p(4,:),obs_p2(k*8-7:k*8,:),1);   %�û�����ĩ�˵�ȥ�Ա����ϰ�������
  [ dis1(k),po1,po2] = dis_compute(eff_p,obs_p2(k*8-7:k*8,:),6); 
         else
            continue;
        end    
 end
    for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==0
     for j=1:3            %������˱������˾����ϰ������С����
         [ dist(j),po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p2(k*8-7:k*8,:),2);
           dis2(k) = min(dist);
     end
        else
            continue;
        end
    end
     dis2 = dis2 - TO_SURFACE_DISTANCE;
     [mind,min_ob2] =min(dis2);
     [mind,min_ob1] =min(dis1);
    dis = min([min(dis1),min(dis2)]);
     if dis >= 0.005                %�ѷ�����ײ
         joint_ang(2*i+5,:) = joint_ang2;
      else
          continue;   
    end

 end
 for i = 1: num_pro-1
    dis1 = ones(1,num_ob)*10000;
    dis2 = ones(1,num_ob)*10000;
    joint_ang2 = (joint_ang(i*2+5,:)+joint_ang(i*2+2+5,:))/2;   
    if norm(joint_ang(i*2+6,:)-joint_ang2)<0.01
        continue;
    end
    for j = 1: 4   %���ĸ��ؽڵ�
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
    end 
    
    joi_p = joi_p/1000;
    
 for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==1
        eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);
   % [ dis1(k),po1,po2] = dis_compute(joi_p(4,:),obs_p2(k*8-7:k*8,:),1);   %�û�����ĩ�˵�ȥ�Ա����ϰ�������
  [ dis1(k),po1,po2] = dis_compute(eff_p,obs_p2(k*8-7:k*8,:),6); 
         else
            continue;
        end    
 end
    for k = 1:num_ob
        if insidepoly3(obs_p(8*k-7,:),m_sur) ==0
     for j=1:3            %������˱������˾����ϰ������С����
         [ dist(j),po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p2(k*8-7:k*8,:),2);
           dis2(k) = min(dist);
     end
        else
            continue;
        end
    end
     dis2 = dis2 - TO_SURFACE_DISTANCE;
     [mind,min_ob2] =min(dis2);
     [mind,min_ob1] =min(dis1);
    dis = min([min(dis1),min(dis2)]);
     if dis >= 0.005                %�ѷ�����ײ
         joint_ang(2*i+6,:) = joint_ang2;
      else
          continue;   
     end
 end
 
  for i = 1: num_joi
      [ T,joi_p(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
      joi_p(i,:) = (bas_R*joi_p(i,:)')'+bap1*1000;
  end
 
   T(1:3,2) = T(1:3,2)*-1;
   T(1:3,3) = T(1:3,3)*-1;
  lastbas_R = bas_R*T(1:3,1:3);
   laststp_R = T(1:3,1:3)';
 % laststp_R = bas_R*lastbas_R';
   laststp_R(1:3,2) =  laststp_R(1:3,2)*-1;
   laststp_R(1:3,3) =  laststp_R(1:3,3)*-1;
   lastjoi=[0,joint_ang(num_ang,4),joint_ang(num_ang,3),joint_ang(num_ang,2),0];%���ؽڽ�˳��ת��Ϊ��һ����׼��
  plot3(joi_p(:,1), joi_p(:,2),joi_p(:,3),'-k','LineWidth',2);%�ú��߻���ĩ��·��
      
     % axis([-1000,1000,-1000,1000,0,500])
end


function[obs_sur] =obp_tran(obp)%�������ϰ����8����ά�ռ��ת��Ϊ�ϰ���Ԫ�أ�5����Ԫ�غ�8���߶�Ԫ�أ�
num_ob = size(obp,1)/8;
obs_sur = zeros(5,4*3,num_ob);
obs_line = zeros(8,2*3,num_ob);
for i = 1:num_ob
    for j=1:4
        if j==4
           obs_sur(j,:,i)=[obp(8*i-4,:),obp(8*i-7,:),obp(8*i-3,:),obp(8*i,:)]; 
        else
           obs_sur(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:),obp(8*i-3+j,:),obp(8*i-4+j,:)];
        end  
    end
    obs_sur(5,:,i)=[obp(8*i-3,:),obp(8*i-2,:),obp(8*i-1,:),obp(8*i,:)];
end
for i=1:num_ob
    for j=1:4
        obs_line(j,:,i)=[obp(8*i-8+j,:),obp(8*i-4+j,:)];
    end
    for j=5:8
        if j==8
            obs_line(j,:,i)=[obp(8*i,:),obp(8*i-3,:)];
        else
            obs_line(j,:,i)=[obp(8*i-8+j,:),obp(8*i-7+j,:)];
        end
    end
end
% for i=1:5
%         for j=1:4
%             sur_x(j) = obs_sur(i,3*j-2,1);
%             sur_y(j) = obs_sur(i,3*j-1,1);
%             sur_z(j) = obs_sur(i,3*j,1);
%         end
%           plot3(sur_x*1000, sur_y*1000,sur_z*1000,'-g','LineWidth',2);
for i = 1: num_ob                                              %��Ҫ���ϰ��ﻭ����ʱ����ϴ˶�
 fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
 patch('Faces',fac_obs,'Vertices',obp(i*8-7:i*8,:)*1000,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',1);
 
         hold on;
end
end

function[obs_bor] = enlarge_border(obs)   %���ϰ���ı߽�����
r = 0.2;
for i = 1:2
    obs_bor(i*4-3,:) = obs(i*4-3,:) + (obs(i*4-3,:)-obs(i*4,:))/norm(obs(i*4-3,:)-obs(i*4,:))*r + (obs(i*4-3,:)-obs(i*4-2,:))/norm(obs(i*4-3,:)-obs(i*4-2,:))*r;
    obs_bor(i*4,:) = obs(i*4,:) + (obs(i*4,:)-obs(i*4-3,:))/norm(obs(i*4,:)-obs(i*4-3,:))*r + (obs(i*4,:)-obs(i*4-1,:))/norm(obs(i*4,:)-obs(i*4-1,:))*r;
    for j = 1:2
        obs_bor(i*4-3+j,:) = obs(i*4-3+j,:) + (obs(i*4-3+j,:)-obs(i*4-2+j,:))/norm(obs(i*4-3+j,:)-obs(i*4-2+j,:))*r + (obs(i*4-3+j,:)-obs(i*4-4+j,:))/norm(obs(i*4-3+j,:)-obs(i*4-4+j,:))*r;
    end
end
end

function [prop] = point_pro_sur(point,poly)      %����ƽ���ϵ�ͶӰ
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
vec = vec/norm(vec);
prop = point-vec*dot(point-poly(1,:),vec);
end

function[vec_repul] = repul_veccompute(obs,map_sur,joi_p)   %�����ϰ���Ի����������Ӱ�죬ȡ�ϰ������ĵ���Ϊ���õ㣬����Ի�����ÿһ���ؽڵ�����������ٶ�����������������,�����ϰ����ͻ����˹ؽڵ�
   obs_aver = zeros(1,3);
   vec_p = zeros(size(joi_p,1),3);
   vec_repul = zeros(1,3);
   %robot_sur = [joi_p(1,:);joi_p(2,:);joi_p(3,:)];
 
   for i= 1:size(obs,1)
       obs_aver = obs_aver + obs(i,:);
   end
   obs_aver = obs_aver/size(obs,1);
    pro_obs = point_pro_sur(obs_aver,map_sur);      %���ϰ���������ϰ����ڱ����ͶӰ����ɲο�ƽ��
    obs_sur = [obs_aver;pro_obs;[0 0 0]];
    for i= 1:4
        joi_pro(i,:) = point_pro_sur(joi_p(i,:),obs_sur);
    end
   
   for i = 1:4%�ϰ�����ĸ��ؽڵ���������
       vec_p(i,:) = (joi_pro(i,:) - obs_aver)/norm(joi_pro(i,:) - obs_aver);
       vec_repul = vec_repul + vec_p(i,:);
   end
      vec_repul = vec_repul + (-obs_aver)/norm(obs_aver); %�ϰ�����ĸ��ؽڵ�����ü����ϰ���Ի���������
      vec_repul = vec_repul/norm(vec_repul);
end

function [state] = insidepoly3(point,poly)        %�жϵ��Ƿ��ڿռ������ڲ�
num_p=size(poly,1);
mean_p=zeros(1,3);
kk=0;
for i=1:num_p
mean_p = mean_p + poly(i,:);
end
mean_p = mean_p/num_p;
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
if abs(dot(point-poly(1,:),vec))>1e-3
    state = 0;
    return;
end
for i=1:num_p
    ls = poly(i,:);
    if i==num_p
    le = poly(1,:);
    else
        le = poly(i+1,:);
    end
    if (dot(cross(point-ls,le-ls),cross(mean_p-ls,le-ls))<0)
         kk=1;
        break;
    end
end
if kk==0
    state =1;
else
    state=0;
end
end
