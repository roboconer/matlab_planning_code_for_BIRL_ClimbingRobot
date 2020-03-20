function [ joint_ang] = motionplan(option,motionp)
%单步运动规划求解
 L1 = 0.3407;
 L2 = 0.2932;
 input_map2;

% motionp =[  1.6105    3.0438    2.6929
%     1.6185    3.0147    2.2495
%     2.2233    2.3869    2.2291];
%        stp = motionp(1,:);
%        bap = motionp(2,:);
%        enp = motionp(3,:);
%       testobj =bap + [0 0 L1];


num_ob = 1;
ob = 0 ;
[A,num_point,num_sur,Pstart,Pend] = surfaces;
sur =zeros(1,50);
num_step = size(motionp,1)-2;

for j = 1:num_step
    stp = motionp(j,:);
    bap = motionp(j+1,:);
    enp = motionp(j+2,:);
    
    
    
if option ==1

%%壁面落足点规划
 
testobj =bap + [0 0 L1];
for i= 1:num_sur
    if  insidepoly3(bap,A{i}) ==1
        B{1} = A{i};
        sur(1) = i;
        break;
    end
end
% if sur(1) == 0
%     return;
% end
%[ obs_p ] = map_obs( B{1},Obs_border{sur(1)},Obs_height{sur(1)});
[ obs_p ] = map_obs( map_border,obs_border,obs_height );
B{1} = map_border;
num_ob = 1;
ob = 0 ;
if size(obs_p,1)>0
for i=1:floor(size(obs_p,1)/8)
    [ dis,po1,po2] = dis_compute(testobj,obs_p(i*8-7:i*8,:),1);
    if dis <= 2*L2+L1
        ob(num_ob) = i;
       % obs_p2(num_ob*8+1:num_ob*8+8,:) = obs_p(i*8-7:i*8,:);
        num_ob = num_ob +1;
    end
    
end
end
if ob ==0
    obs_p2 = zeros(4,3);
else
    
obs_p2 = zeros(length(ob)*8,3);
for i = 1:length(ob)
    obs_p2(i*8-7:i*8,:) = obs_p(ob(i)*8-7:ob(i)*8,:);
end
end
 
%[joint_ang ] = colli_avoidance( map_border,stp,enp,bap,obs_p2 );
joint_ang{j} = colli_avoidance(  B{1},stp,enp,bap,obs_p2 );

else            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%壁面过渡运动规划
    
    %%壁面过渡运动规划
    testobj =bap + [0 0 L1];
    num_obsur = 3;
  %  obs_p = 0;
for i= 1:num_sur
    if  insidepoly3(bap,A{i}) ==1
        B{1} = A{i};
        sur(1) = i;
    end
    if insidepoly3(enp,A{i}) ==1
          B{2} = A{i};
          sur(2) = i;
    end
     if insidepoly3(stp,A{i}) ==1
          B{3} = A{i};
          sur(3) = i;
     end       
end
if any(sur(1:3) == 0)
    joint_ang = 0;
      h=msgbox('没有求解到落足点对应的壁面','warn'); 
          uiwait(h,2);
    return;
end

for i = 1:num_sur 
    [ dis,po1,po2] = dis_compute(testobj,A{i},3);
    if dis <= 2*L2+L1 
        if any(sur(1:3)==i)
            continue;
        else
            sur(num_obsur +1) = i;
            B{num_obsur+1} = A{i};
            num_obsur = num_obsur +1;
        end
    end
end
num_ob = 1;
for i = 1:num_sur                            %求解机器人工作空间内的有效障碍物
    if Obs_height{i} == 0
        continue;
    end
    num_ob2 = length(Obs_height{i});
    obs_p(num_ob*8-7:(num_ob + num_ob2 - 1)*8,:)  = map_obs( A{i},Obs_border{i},Obs_height{i} );
    % obs_p(1:8,:) = map_obs( A{i},Obs_border{i},Obs_height{i} );
     num_ob = num_ob + num_ob2;
end
 %obs_p
num_ob = 1;
 if size(obs_p,1)>0
for i=1:floor(size(obs_p,1)/8)
    [ dis,po1,po2] = dis_compute(testobj,obs_p(i*8-7:i*8,:),1);
    if dis <= 2*L2+L1
        ob(num_ob) = i;
       % obs_p2(num_ob*8+1:num_ob*8+8,:) = obs_p(i*8-7:i*8,:);
        num_ob = num_ob +1;
    end
end
 end
if ob ==0
    obs_p2 = zeros(4,3);   %表示工作空间内无障碍物
else  
obs_p2 = zeros(length(ob)*8,3);
for i = 1:length(ob)
    obs_p2(i*8-7:i*8,:) = obs_p(ob(i)*8-7:ob(i)*8,:);
end
end
%  B
% B={A{1},A{2},A{1}};
joint_ang{j} = colli_avoidance2( B,stp,enp,bap,obs_p2,1 );
end
end
end