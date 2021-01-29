function [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option)
%  ��������ɵ��������˶�����Ի������ڲ�ͬ�������ɵ����,��Ҫ����������ڱ��棬Ŀ����������ڱ��棬��ʼ��������ڱ���Ȳ���
%[ joint_ang,p_s ] = joint_angle(stp,enp,bap);

%%% ��ʱ
tic
disp('Ԥ������')

CREATE_G_CODE = 0;
DRAW = 1    ;%1�ǻ�ͼ��0�ǲ���
global BandClamp GModule HollowPart IModule TModuleP1 TModuleP2 SModule;
[BandClamp,GModule,HollowPart,IModule,TModuleP1,TModuleP2,SModule] = readAllParts();
CurHMatrix  = eye(4);
dis_warn = 0.05;  %ע�����Ϊ5cm
dis_safe = 0.02;  %��С��ȫ����Ϊ2cm
po1 = zeros(1,3);
po2 = zeros(1,3);
vec_lin = zeros(1,3);
vec_eff = zeros(1,3);
num_up = 0;
enp_o = enp;
stp_o = stp;
bap_o = bap;
TO_SURFACE_DISTANCE = 0.15;%�����ľ���
wrobot = struct('DoFs',5,'LinksLen',[329,293.2,293.2,329],'ModuleRadii',50,...
'CurGraspMtx',CurHMatrix,'CurJAs',[0   20    -40   20 0],'FixedGripper',1,'TarGraspMtx',eye(4,4),...
'TarJAs',[0,0,0,0,0],'hIKine',@IKine5D,'hLink',@Linkage5D);
jmax = [360,210,120,210,360];    %�ؽڽǱ仯�����ֵ
jmin = [-360,-30,-120,-30,-360];  %�ؽڽǱ仯����Сֵ
wrobot.CurJAs = fir_joi;
num_msur=size(M_sur,2);   %��������˶���������Ҫ�����ı��漰������ϵ�����Ի������ڻ�����ϵ��ת�������һ������Ϊ���������ڱ��棬�ڶ�������ΪĿ����������ڱ��棬����������Ϊ��ʼ��������ڱ���
m_sur = cell(num_msur,1);        %����ε��������ϵת����Ĵ���
num_ob = floor(size(obs_p,1)/8);
obs_p2 = obs_p;
wrobot.CurGraspMtx(1:3,4) = bap*1000;
wrobot.CurGraspMtx(1:3,1:3) = bas_R;
%%% ��ͼ
if DRAW
    figure (3);
    for i = 1:num_msur    %�����ͼ
        pp = patch(M_sur{i}(:,1)*1000,M_sur{i}(:,2)*1000,M_sur{i}(:,3)*1000,[0.467,0.533,0.6],'facealpha',0.3,'LineWidth',1);
    end
    hold on 
    if num_ob>=1          %�ж��ϰ����Ƿ����
        obp_tran(obs_p);      %�ϰ����ͼ
    end
    plot3(stp(1,1)*1000,stp(1,2)*1000,stp(1,3)*1000,'*g','LineWidth',2);
    plot3(enp(1,1)*1000,enp(1,2)*1000,enp(1,3)*1000,'*g','LineWidth',2);
    axis equal
end
      
for i=1:num_msur
    num_mp(i)=size(M_sur{i},1);
end
Vnm = zeros(num_msur,3);
for i=1:num_msur
    Vnm(i,:)=cross(M_sur{i}(2,:)-M_sur{i}(1,:),M_sur{i}(3,:)-M_sur{i}(2,:));   %�������η�����
    Vn_norm=norm(Vnm(i,:));
    Vnm(i,:)=Vnm(i,:)/Vn_norm;
end
bas_x = (M_sur{1}(2,:)-M_sur{1}(1,:))/norm(M_sur{1}(2,:)-M_sur{1}(1,:));   %��������˶��Ļ�����ϵ
bas_z = Vnm(1,:);
bas_y = cross(bas_z,bas_x); 
% bas_R = [bas_x',bas_y',bas_z'];
tar_x = (M_sur{2}(2,:)-M_sur{2}(1,:))/norm(M_sur{2}(2,:)-M_sur{2}(1,:));   %��������˶���Ŀ������ϵ
tar_z = Vnm(2,:);
tar_y = cross(tar_z,tar_x);
tar_R = [tar_x',tar_y',tar_z'];
tar_to_bas_R = bas_R'*tar_R;    %Ŀ������ϵ�ڻ�����ϵ�µı�ʾ
wor_to_bas_R = bas_R';   %��������ϵ�ڻ�����ϵ�ı�ʾ
wtoba_p = (-wor_to_bas_R*bap')';

for i = 1:num_msur                  %����������˶����̸��ŵı���
    for j = 1:num_mp(i)
        m_sur{i}(j,:) = (wor_to_bas_R*M_sur{i}(j,:)')' + wtoba_p; 
    end
end
if num_ob>=1            %�������ϵ�µ��ϰ���
    for i = 1:size(obs_p,1)
        obs_p(i,:) = (wor_to_bas_R*obs_p(i,:)')'+ wtoba_p;
    end
end




enp = (wor_to_bas_R*enp')'+ wtoba_p;   %Ŀ����ڻ�����ϵ�µı�ʾ
stp = (wor_to_bas_R*stp')'+ wtoba_p;   %��ʼ������ڻ������µı�ʾ
enpp =enp;
enp = enp + (wor_to_bas_R*Vnm(2,:)')'*TO_SURFACE_DISTANCE;       %Ŀ��������ĸ�����
sta_x = (M_sur{3}(2,:)-M_sur{3}(1,:))/norm(M_sur{3}(2,:)-M_sur{3}(1,:));   %��������˶����������ϵ
sta_z = Vnm(3,:);
sta_y = cross(sta_z,sta_x);
sta_R = stp_R;                   %�������ϵ�ڻ�����ϵ�µı�ʾ
wrobot.TarGraspMtx(1:3,1:3) = stp_R;
Vn = (wor_to_bas_R*Vnm(3,:)')';
%     wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
%     wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %����ĩ������ϵ����  
wrobot.TarGraspMtx(1:3,4) = (stp )'*1000;
[ Flag,sta_ang] = IKine5DNew(wrobot);                %���������ʼ���͵Ĺؽڽ�
un_sta = sta_ang(1,5); %��¼��˵�I�ؽڽǶ�
wrobot.TarGraspMtx(1:3,4) = enpp'*1000;        %�������Ŀ�깹�͵Ĺؽڽ�
wrobot.CurJAs = sta_ang;
wrobot.TarGraspMtx(1:3,1:3) = tar_to_bas_R;
wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %����ĩ������ϵ����
% [Flag, end_ang] = IKine5DNew(wrobot);   
[Flag, end_ang] = IKine5D22(wrobot);
if Flag == 0 
    h=msgbox('Ŀ������ɲ��ɹ�','warn'); 
    uiwait(h,2);
    return;
end
wrobot.TarGraspMtx(1:3,4) = enp'*1000;        %�������Ŀ�깹�͵Ĺؽڽ�
wrobot.CurJAs = sta_ang;  
[Flag, end_ang] = IKine5DNew(wrobot);   
if Flag == 0 
    h=msgbox('����Ŀ������ɲ��ɹ�','warn'); 
    uiwait(h,2);
    return;
end  
if so_option == 2
   [Flag, end_ang] = IKine5D22(wrobot);
   Flag;
end
wrobot.TarJAs =sta_ang;
wrobot.TarJAs =end_ang;


toc
%%% ��ʱend

%%% ��ʱbegin
tic
disp('�û���������5cm')

for i = 1:TO_SURFACE_DISTANCE * 100             %���û���������5cm,������ʼ���Ŀ��������һ�ξ�������ƽ���������� 
    %         wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.008)')*1000;
    wrobot.TarGraspMtx(1:3,4) = ((stp + Vn*i*0.01)')*1000;
    wrobot.TarGraspMtx(1:3,1:3) = sta_R;
%      wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
%      wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %����ĩ������ϵ����
    if i > 1 
        wrobot.CurJAs = joint_ang(i-1,:);
    end
    [Flag, joint_ang(i,:)] = IKine5DNew(wrobot);
    if Flag == 0
            h=msgbox('��ʼ����λ������','warn'); 
            uiwait(h,2);
            break;
        joint_ang(i,:) = joint_ang(i-1,:) + [0 3 0 0 0];
    end
end 
toc
%%% ��ʱend

% %%% ��ʱbegin
% tic
% disp('���̹滮')

stp = stp + Vn * TO_SURFACE_DISTANCE;
num_ang = floor(TO_SURFACE_DISTANCE * 100);   
while norm(joint_ang(num_ang,:)-end_ang)>1e-4
    
    %%%%%%%%%%%%%%������ǰĩ�˵�
    [ T,joi_pp ] =  Kine5D( joint_ang(num_ang,:) ,4);
    joi_pp = (bas_R*joi_pp')'+ bap_o*1000;
    %%% ��ͼ
    if DRAW
        plot3(joi_pp(1,1), joi_pp(1,2),joi_pp(1,3),'om','LineWidth',1);
    end
    joi_diffe = abs(end_ang - joint_ang(num_ang,:));   %ÿ���ؽڽ�֮��Ĳ�ֵ�㹻С��ֱ�ӵ���Ŀ���
    if joi_diffe(1:4) < 3
        num_ang = num_ang +1;
        joint_ang(num_ang,:) = end_ang;
        break;
    end
    
    %%%%%%%%%%%%�滮�������޽��
    if num_ang>200   
        if so_option == 1
            [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
            return;
        else
            h=msgbox('�滮�������޽��','warn'); 
            uiwait(h,2);
            break;
        end
    end
    
    dis1 = 10000;
    dis2 = 10000;
    vec_lin = zeros(1,3);
    vec_eff = zeros(1,3);
    
    %%% ��ʱ
    tic
    disp('���ĸ��ؽڵ�')
    
    for j = 1: 4   %���ĸ��ؽڵ�
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang(num_ang,:) ,j);       
    end 
    
    toc
    %%% ��ʱ
    
    %%% ��ʱ
    tic
    disp('ĩ���Ƿ�����������')
    
    joi_p = joi_p/1000;
    joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
    eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,20);
    eff_p2 = cir_seem_poly( joi_lap,joi_p(4,:)-joi_p(3,:),0.2,10);
         
    if norm(joi_p(4,:))<0.4  %ĩ�˾����������
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + [(end_ang(1)-joint_ang(num_ang,1))/abs(end_ang(1)-joint_ang(num_ang,1))*3,0,0,0,0];
        joint_ang(num_ang+1,:) = joint_ang(num_ang+1,:)+  [0,-2,4,-2,0];
        for i = 1:5             %�жϹ滮�õ��Ĺؽڽ�ֵ��û��������Χ����������Ϊ��һ״̬��ֵ
            if joint_ang(num_ang+1,i)+2>jmax(i) || joint_ang(num_ang+1,i)-2<jmin(i)
               joint_ang(num_ang+1,i) = joint_ang(num_ang,i);
            end
        end
        num_ang = num_ang + 1; 
        continue;
    end
    if  norm(joi_p(4,:)-enp)<0.2         %���������������ʧЧʱ����
        [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
        num_inter = floor(max_diffe/2);
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
        num_ang = num_ang + 1; 
        continue;
    end
    
    toc
    %%% ��ʱ
    
    
    %%% ��ʱ
    tic
    disp('������˱������˾���������С����')
    
    %������˱������˾���������С����
    for i= 1: num_msur
        for j=1:3            
            if j == 3
                [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],m_sur{i},4);
            else
                [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],m_sur{i},4);
            end
            if dist < dis1
                dis1 = dist;
                vec_lin = po1 - po2;
            end
        end 
    end
        toc
    %%% ��ʱ
    
    %%% ��ʱ
    tic
    disp('������˱������˵��ϰ������С����')
    %������˱������˵��ϰ������С����
    if num_ob >0    
        for i = 1:num_ob
            for j = 1:3       
                if j == 3
                    [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],obs_p(i*8-7:i*8,:),2);
                else
                    [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p(i*8-7:i*8,:),2);
                end
                if dist < dis1
                    dis1 = dist;
                    vec_lin = po1 - po2;
                end
            end
        end
    end
    dis1 = dis1 - 0.05;
    
    toc
    %%% ��ʱ
    
    %%% ��ʱ
    tic
    disp('�������ĩ�˾����ϰ������С����')
    
    %�������ĩ�˾����ϰ������С����
    if num_ob >0            
        for i = 1:num_ob 
             [ dist,po1,po2] = dis_compute(joi_p(4,:),obs_p(i*8-7:i*8,:),1);
             dist = dist - 0.18;
             if dist < dis2
                  dis2 = dist;
                  vec_eff = po1 - po2;
             end
        end
    end
    toc
    %%% ��ʱ
    
    %%% ��ʱ
    tic
    disp('�������ĩ�˾���������С����')
     %�������ĩ�˾���������С����
    for i= 1: num_msur                
        [ dist,po1,po2] = dis_compute(eff_p,m_sur{i},5);
        if dist < dis2 
            dis2 = dist ;
            vec_eff = po1 - po2;
        end
    end 
    toc
    %%% ��ʱ
    
    %%% ��ʱ
    tic
    disp('�����滮')
    
    [state(num_ang-4),vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,joi_p(4,:),enp);
    if state(num_ang-4) == 0
        if so_option == 1
            [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
            return;
            else
            h=msgbox('������ײ���滮ʧ��','warn'); 
            uiwait(h,2);
            break;
        end
    elseif state(num_ang-4) == 1
        [max_diffe,num_j]=max(abs(end_ang(1:4)-joint_ang(num_ang,1:4)));
        num_inter = floor(max_diffe/3);
        joint_ang(num_ang+1,:) = joint_ang(num_ang,:) + (end_ang-joint_ang(num_ang,:))/num_inter;
    for i = 1:5             %�жϹ滮�õ��Ĺؽڽ�ֵ��û��������Χ����������Ϊ��һ״̬��ֵ
        if joint_ang(num_ang+1,i)+2>jmax(i) || joint_ang(num_ang+1,i)-2<jmin(i)
            joint_ang(num_ang+1,i) = joint_ang(num_ang,i);
        end
    end
        num_ang = num_ang + 1; 
    elseif state(num_ang-4) == 2     
        next_p = joi_p(4,:) + vec_re * 0.01;
        wrobot.TarGraspMtx(1:3,4) = (next_p')*1000;
        wrobot.TarGraspMtx(1:3,1:3) = T_tb(1:3,1:3);
        wrobot.CurJAs=joint_ang(num_ang,:);
        [Flag, joint_ang(num_ang+1,:)] = IKine5DNew(wrobot);
        if Flag == 0
            num_up = num_up +1;
            if num_up == 10
                if so_option == 1
                    [lastbas_R,laststp_R,lastjoi,joint_ang] = colli_avoidance2( M_sur,stp_o,enp_o,bap_o,obs_p2,bas_R,stp_R,fir_joi,2);
                    return;
                else
                     h=msgbox('�����滮ʧ��','warn'); 
                     uiwait(h,2);
                     break;
                end
            else
                joint_ang(num_ang+1,:) =  joint_ang(num_ang,:) +[0 2 0 0 0];
            end
        end
        for i = 1:5             %�жϹ滮�õ��Ĺؽڽ�ֵ��û��������Χ����������Ϊ��һ״̬��ֵ
            if joint_ang(num_ang+1,i)+2>jmax(i) || joint_ang(num_ang+1,i)-2<jmin(i)
                joint_ang(num_ang+1,i) = joint_ang(num_ang,i);
            end
        end
        num_ang = num_ang + 1;
    end
    toc
    %%% ��ʱ
end
    disp('������')
    num_ang
% toc
% %%% ��ʱend

%%% ��ʱbegin
tic
disp('�½����̹滮')
%  if CREATE_G_CODE ~= 1
%      for i = 1:(TO_SURFACE_DISTANCE-0.01) * 100-1             %ʹ������ĩ�˵���Z�᷽�򿿽�TO_SURFACE_DISTANCE-0.01��ÿ��1cm
%           [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
%           end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.01*1000;
%           wrobot.TarGraspMtx(1:3,4) = end_p';
%           wrobot.CurJAs=joint_ang(num_ang+i-1,:);
%            wrobot.TarGraspMtx(1:3,1:3) =  T(1:3,1:3);
%        % wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
%        % wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %����ĩ������ϵ����
%            [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(wrobot);
%            if Flag ==0
%                 h=msgbox('Ŀ����½��޽�','warn'); 
%                          uiwait(h,2);
%                 return;
%            end
%      end
%      num_ang = num_ang + (TO_SURFACE_DISTANCE-0.01) * 100 - 1;
%     for i = 1:2               %ʹ������ĩ�˵���Z�᷽�򿿽�1cm��ÿ��0.5cm
%           [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
%           end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.005*1000;
%           wrobot.TarGraspMtx(1:3,4) = end_p';
%           wrobot.CurJAs=joint_ang(num_ang+i-1,:);
%            wrobot.TarGraspMtx(1:3,1:3) =  T(1:3,1:3);
%           [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(wrobot);
%      end      
%            num_ang = num_ang + 2;
%  end

 if CREATE_G_CODE ~= 1
     for i = 1:TO_SURFACE_DISTANCE * 100             %ʹ������ĩ�˵���Z�᷽�򿿽�TO_SURFACE_DISTANCE-0.01��ÿ��1cm
          [ T,end_p] = Kine5D( joint_ang(num_ang-1+i,:) ,4);
          end_p = end_p - (wor_to_bas_R*Vnm(2,:)')'*0.01*1000;
          wrobot.TarGraspMtx(1:3,4) = end_p';
          wrobot.CurJAs=joint_ang(num_ang+i-1,:);
           wrobot.TarGraspMtx(1:3,1:3) =  T(1:3,1:3);
       % wrobot.TarGraspMtx(1:3,2)=wrobot.TarGraspMtx(1:3,2)*-1;
       % wrobot.TarGraspMtx(1:3,3)=wrobot.TarGraspMtx(1:3,3)*-1;    %����ĩ������ϵ����
           [Flag, joint_ang(num_ang+i,:)] = IKine5DNew(wrobot);
           if Flag ==0
                h=msgbox('Ŀ����½��޽�','warn'); 
                         uiwait(h,2);
                return;
           end
     end
     num_ang = num_ang + TO_SURFACE_DISTANCE * 100;
 end

toc
%%% ��ʱend

%%%%���һ֡����Ԥ�������
joint_ang = [sta_ang;joint_ang];
num_ang = num_ang + 1;
joint_ang(:,5) = un_sta;
num_joi = size(joint_ang,1);

wrobot.TarJAs =joint_ang(1,:);
%%% ��ͼ
if DRAW
    DrawRobotmo(wrobot,1);
end
wrobot.TarJAs =joint_ang(num_ang,:);
%%% ��ͼ
if DRAW
    DrawRobotmo(wrobot,1);
end

%%% ��ʱbegin
tic
disp('ƽ��·��')

 num_pro = num_joi - 11;                  %%��·����ƽ������
 num_pro = floor(num_pro/2);
 for i = 1: num_pro
     dis1 = 10000;
    dis2 = 10000;
    vec_lin = zeros(1,3);
    vec_eff = zeros(1,3);
    joint_ang2 = (joint_ang(i*2-1+5,:)+joint_ang(i*2+1+5,:))/2;
    
    if norm(joint_ang(i*2+5,:)-joint_ang2)<0.01
        continue;
    end
    for j = 1: 4   %���ĸ��ؽڵ�
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
    end 
    
    joi_p = joi_p/1000;
    joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
    eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,10);
      for k= 1: num_msur
         for j=1:3            %������˱������˾���������С����
             if j == 3
                 [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],m_sur{k},4);
             else
             [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],m_sur{k},4);
             end
         if dist < dis1
         dis1 = dist;
         vec_lin = po1 - po2;
         end
         end 
      end
      if num_ob >0
          for k = 1:num_ob
      for j = 1:3       %������˱������˵��ϰ������С����
          if j == 3
              [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],obs_p(k*8-7:k*8,:),2);
          else
           [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p(k*8-7:k*8,:),2);
          end
         if dist < dis1
            dis1 = dist;
            vec_lin = po1 - po2;
         end
      end
          end
      
      end
      dis1 = dis1 - 0.05;
if num_ob >0
     for k = 1:num_ob 
       
            [ dist,po1,po2] = dis_compute(joi_p(4,:),obs_p(k*8-7:k*8,:),1);
            dist = dist - 0.2;
         if dist < dis2
             dis2 = dist;
              vec_eff = po1 - po2;
         end
     end
end       
      for k= 1: num_msur
          [ dist,po1,po2] = dis_compute(eff_p,m_sur{k},5);
         if dist < dis2 
         dis2 = dist ;
         vec_eff = po1 - po2;
         end
      end 
      [state1,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,joi_p(4,:),enp);
      if state1 > 0
          joint_ang(2*i+5,:) = joint_ang2;
      else
          continue;
      end
 end
 for i = 1: num_pro-1
     dis1 = 10000;
    dis2 = 10000;
    vec_lin = zeros(1,3);
    vec_eff = zeros(1,3);
    joint_ang2 = (joint_ang(i*2+5,:)+joint_ang(i*2+2+5,:))/2;
    
    if norm(joint_ang(i*2+1+5,:)-joint_ang2)<0.01
        continue;
    end
    for j = 1: 4   %���ĸ��ؽڵ�
         [ T_tb,joi_p(j,:)] =  Kine5D(joint_ang2 ,j);       
    end 
    
    joi_p = joi_p/1000;
    joi_lap = joi_p(4,:) + (joi_p(3,:)-joi_p(4,:))/norm(joi_p(3,:)-joi_p(4,:))*0.06;
    eff_p = cir_seem_poly( joi_p(4,:),joi_p(4,:)-joi_p(3,:),0.2,10);
      for k= 1: num_msur
         for j=1:3            %������˱������˾���������С����
             if j == 3
                 [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],m_sur{k},4);
             else
             [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],m_sur{k},4);
             end
         if dist < dis1
         dis1 = dist;
         vec_lin = po1 - po2;
         end
         end 
      end
      if num_ob >0
          for k = 1:num_ob
      for j = 1:3       %������˱������˵��ϰ������С����
          if j == 3
              [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_lap],obs_p(k*8-7:k*8,:),2);
          else
           [ dist,po1,po2] = dis_compute([joi_p(j,:);joi_p(j+1,:)],obs_p(k*8-7:k*8,:),2);
          end
         if dist < dis1
            dis1 = dist;
            vec_lin = po1 - po2;
         end
      end
          end
      
      end
      dis1 = dis1 - 0.05;
if num_ob >0
     for k = 1:num_ob 
       
            [ dist,po1,po2] = dis_compute(joi_p(4,:),obs_p(k*8-7:k*8,:),1);
            dist = dist - 0.2;
         if dist < dis2
             dis2 = dist;
              vec_eff = po1 - po2;
         end
     end
end       
      for k= 1: num_msur
          [ dist,po1,po2] = dis_compute(eff_p,m_sur{k},5);
         if dist < dis2 
         dis2 = dist ;
         vec_eff = po1 - po2;
         end
      end 
      [state1,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,joi_p(4,:),enp);
      if state1 > 0
          joint_ang(2*i+1+5,:) = joint_ang2;
      else
          continue;
      end
 end
  for i = 1: num_joi
      [ T,joi_po(i,:) ] =  Kine5D( joint_ang(i,:) ,4);
       joi_po(i,:) = (bas_R*joi_po(i,:)')'+bap_o*1000;
  end
  
toc
%%% ��ʱend

T(1:3,2) = T(1:3,2)*-1;
T(1:3,3) = T(1:3,3)*-1;
lastbas_R = bas_R*T(1:3,1:3);

laststp_R = T(1:3,1:3)';
laststp_R(1:3,2) =  laststp_R(1:3,2)*-1;
laststp_R(1:3,3) =  laststp_R(1:3,3)*-1;
lastjoi=[0,joint_ang(num_ang,4),joint_ang(num_ang,3),joint_ang(num_ang,2),0];
%%% ��ͼ
    if DRAW
        plot3(joi_po(:,1), joi_po(:,2),joi_po(:,3),'-g','LineWidth',2);
        axis([0,3000,-1500,-500, 0,2500])
    end
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

for i = 1: num_ob 
 fac_obs = [1,2,3,4;5,6,7,8;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8];
 patch('Faces',fac_obs,'Vertices',obp(i*8-7:i*8,:)*1000,'FaceVertexCData',[0.5 0.987 0],'FaceColor','flat','facealpha',0.3);
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
   
   for i = 1:4
       vec_p(i,:) = (joi_pro(i,:) - obs_aver)/norm(joi_pro(i,:) - obs_aver);
       vec_repul = vec_repul + vec_p(i,:);
   end
      vec_repul = vec_repul + (-obs_aver)/norm(obs_aver); 
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
if abs(dot(point-poly(1,:),vec))>1e-5
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

function[state,vec_re] = repadd_veccom(dis1,dis2,vec_lin,vec_eff,eff_p,enp)   %��������˱�����������ͻ�����ĩ�˳���������Ŀ�������Ի������˶���Ӱ�죬���뱾����С�����ĩ����С���룬��ǰĩ�˵��Ŀ���
   vec_re = zeros(1,3);
    dis_warn = 0.05;  %ע�����Ϊ5cm
  dis_safe = 0.02;  %��С��ȫ����Ϊ2cm
  vec_lin = vec_lin/norm(vec_lin);
  vec_eff = vec_eff/norm(vec_eff);
  vec_tar = (enp - eff_p)/norm(enp - eff_p);
     if dis1 <= 0.003 || dis2 <= 0.003
         state = 0 ;  %������ײ
         return;
     end
     if dis1 >= dis_warn && dis2 >= dis_warn
         state = 1;  %û�з�����ײ
         return;
     end
     if dis1 <dis_warn && dis1 >=dis_safe
           rec_lin =(  0.1*(1/dis1-1/0.05))* vec_lin;
     elseif dis1 >0 && dis1 < dis_safe
         rec_lin = 3 * vec_lin;
     elseif dis1 >= dis_warn
         rec_lin = 0;
     end
     if dis2 <dis_warn && dis2 >= dis_safe
           rec_eff =(  0.1*(1/dis2-1/0.05))* vec_eff;
     elseif dis2 >0&& dis2 < dis_safe
         rec_eff = 3 * vec_eff;
     elseif dis2 >= dis_warn
         rec_eff = 0;
     end
     state = 2;    %����ע����뷶Χ��
      vec_re =  vec_tar + rec_lin + rec_eff ;
     % vec_re = rec_lin + rec_eff ;
      vec_re = vec_re/norm(vec_re);
end
function [avp] = average_p(obp)%���ϰ������ĵ�
avp = zeros(1,3);
for i = 1:size(obp,1)
    avp = avp + obp(i,:);
end
   avp = avp/size(obp,1); 
end