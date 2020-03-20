function [ se_footstep3d,step_op ] = surfaces_astar(x)
%输入：过渡位置，起始点目标点，障碍物位置，高度(后三个参数由surfaces输入)
%输出：无碰落足点序列
%2020.1.12:将输出se_footstep3d改成元胞数组
%   多壁面的离散无碰落足点规划求解
[A1,num_point,num_sur,Pstart,Pend] = surfaces;
r=0.1998;   %吸盘半径
[map_border, Obs_border, Obs_height] = input_map2;
Vz=zeros(num_sur,3);
Vy=zeros(num_sur,3);
Vx=zeros(num_sur,3);
pw = zeros(num_sur,3);
psw= zeros(num_sur,3);
% Pstart = zeros(1,3);
% Pend = zeros(1,3);
R_sw = zeros(3,3,num_sur);%平面坐标系在世界坐标系下的转换矩阵
R_ws = zeros(3,3,num_sur);%世界坐标系在平面坐标系下的转换矩阵
Xa=[1,0,0];
Ya=[0,1,0];
Za=[0,0,1];

%第一个for循环：将壁面顶点扩张，效果为：膨胀一个吸盘半径
for i=1:num_sur
    %沿1、n顶点的对角线延长
    v1= (A1{i}(num_point(i),:)-A1{i}(1,:))/norm(A1{i}(num_point(i),:)-A1{i}(1,:));
    v2= (A1{i}(2,:)-A1{i}(1,:))/norm(A1{i}(2,:)-A1{i}(1,:));
    ang_cos = dot(v1,v2);
    ang_sin = sqrt(1-ang_cos^2);
    l = r/ang_sin;
    A{i}(1,:) = A1{i}(1,:)+ v1*l +v2*l;
    
    v1=( A1{i}(1,:)-A1{i}(num_point(i),:))/norm(A1{i}(1,:)-A1{i}(num_point(i),:));
    v2= (A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:))/norm(A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:));
    ang_cos = dot(v1,v2);
    ang_sin = sqrt(1-ang_cos^2);
    l = r/ang_sin;
    A{i}(num_point(i),:) = A1{i}(num_point(i),:)+ v1*l +v2*l;
    
   %沿剩下顶点的对角线延长
   for j=2:num_point(i)-1
       v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
       v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
       ang_cos = dot(v1,v2);
       ang_sin = sqrt(1-ang_cos^2);
       l = r/ang_sin;
       A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;   
   end
end

%第二个for：赋值
for i= 1:num_sur
    B{i} = A{i};
end

%第三个for:计算各个壁面法向量
for i=1:num_sur
    Vz(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量,建立空间坐标系
    Vz_norm=norm(Vz(i,:));
    Vz(i,:)=Vz(i,:)/Vz_norm;
end

%第四个for：算出每个壁面的y轴方向（x轴为第一条边，z轴为法向量），定好壁面坐标系
for i = 1:num_sur
    Vx(i,:) = A{i}(2,:)-A{i}(1,:);
    Vx_norm=norm(Vx(i,:));
    Vx(i,:)=Vx(i,:)/Vx_norm;
    Vy(i,:)=cross(Vz(i,:),Vx(i,:));
end

%第五个for：计算每个壁面的旋转矩阵
%R_sw平面坐标系在世界坐标系下的转换矩阵
%R_ws世界坐标系在平面坐标系下的转换矩阵
for i =1:num_sur
R_sw(:,:,i)=[dot(Vx(i,:),Xa),dot(Vy(i,:),Xa),dot(Vz(i,:),Xa);dot(Vx(i,:),Ya),dot(Vy(i,:),Ya),dot(Vz(i,:),Ya);dot(Vx(i,:),Za),dot(Vy(i,:),Za),dot(Vz(i,:),Za)]; 
R_ws(:,:,i) = R_sw(:,:,i)';
end

%第六个for：确定每个壁面的原点
for i=1:num_sur
    pw(i,:) = A{i}(1,:);%每个壁面的原点都是每个壁面的第一个点
    psw(i,:) =(-R_ws(:,:,i)*pw(i,:)')';  %世界坐标系的原点在平面坐标系上的表示
end

%第七个for：将每个壁面的顶点都转换到平面坐标系下
for i=1:num_sur
    for j=1:num_point(i)
        B{i}(j,:) = (R_ws(:,:,i)*A{i}(j,:)')'+psw(i,:);
    end
end

x=[ Pstart;
     x
    Pend];

%   x=[3.9499    7.1959    2.0064    
%     4.3380    7.2582    2.7340];%仿真重要数据
% x = [ 2.2500    2.0250    0.4000
%     2.5341    2.7056    0.4000
%     3.0257    3.2951    1.1057
%     2.8237    3.5835    2.2022
%     1.9189    3.2551    2.2495
%     1.6185    3.0147    2.2495
%     2.2233    2.3869    2.2291
%     1.7667    1.9667    1.5667];
% ps = [ 4.8050    1.9302    3.7601];
% ps =Pstart;
%     
%  pe=Pend;
 
num_op = 0;

%for i=1:2 %mark：这里我不知道2是代表什么，照理说应该是壁面数量才对的；将这里先改成num_sur试试
se_footstep3d = {}
% for i=1:num_sur
for i=1:num_sur
    ps=x(2*i-1,:);%第i个壁面的开始点
    pe=x(2*i,:);%第i个壁面的终点
    ps=(R_ws(:,:,i)*ps')'+psw(i,:);
    pe=(R_ws(:,:,i)*pe')'+psw(i,:);
    map_border=zeros(num_point(i),2);
    for j=1:num_point(i)
        map_border(j,:) = B{i}(j,1:2);%在平面坐标系下，因此只需取两个坐标
    end
    % 返回的落足点序列应该是包括始末点的
    [ se_footstep3d{1,i},numst ] = footstepsolution( ps(1,1:2),pe(1,1:2),map_border,Obs_border{i},Obs_height{i});
    nums = numst +1;%步数（包括起始点的全部落足点数-1）
    step_op(num_op+1:num_op+nums-2) = 1;
    step_op(num_op+nums-1:num_op+nums) = 2;%包括终点，最后两个点都是标2的
    num_op = num_op + nums;
    for k=1:numst+1
         se_footstep3d{1,i}(k,:) = (R_sw(:,:,i)*se_footstep3d{1,i}(k,:)')'+pw(i,:);
    end
    se_footstep3d{1,i}
    figure(1)
    Vn=Vz(i,:);
    DrawRobotfoot(se_footstep3d{1,i},Vn);
    plot_obs( map_border,Obs_border{i},Obs_height{i},R_sw(:,:,i),pw(i,:) );
    
 end
 
%  for i=1:numst+1
%      se_footstep3d(i,:) = (R_sw(:,:,2)*se_footstep3d(i,:)')'+pw(2,:);
%  end
%  figure(1)
%  Vn=Vz(2,:);
%  DrawRobotfoot(se_footstep3d,B{2},obs_border,obs_height,Vn);
end

