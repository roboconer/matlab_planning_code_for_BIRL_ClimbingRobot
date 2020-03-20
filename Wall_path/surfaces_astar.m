function [ se_footstep3d,step_op ] = surfaces_astar(x)
%���룺����λ�ã���ʼ��Ŀ��㣬�ϰ���λ�ã��߶�(������������surfaces����)
%������������������
%2020.1.12:�����se_footstep3d�ĳ�Ԫ������
%   ��������ɢ���������滮���
[A1,num_point,num_sur,Pstart,Pend] = surfaces;
r=0.1998;   %���̰뾶
[map_border, Obs_border, Obs_height] = input_map2;
Vz=zeros(num_sur,3);
Vy=zeros(num_sur,3);
Vx=zeros(num_sur,3);
pw = zeros(num_sur,3);
psw= zeros(num_sur,3);
% Pstart = zeros(1,3);
% Pend = zeros(1,3);
R_sw = zeros(3,3,num_sur);%ƽ������ϵ����������ϵ�µ�ת������
R_ws = zeros(3,3,num_sur);%��������ϵ��ƽ������ϵ�µ�ת������
Xa=[1,0,0];
Ya=[0,1,0];
Za=[0,0,1];

%��һ��forѭ���������涥�����ţ�Ч��Ϊ������һ�����̰뾶
for i=1:num_sur
    %��1��n����ĶԽ����ӳ�
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
    
   %��ʣ�¶���ĶԽ����ӳ�
   for j=2:num_point(i)-1
       v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
       v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
       ang_cos = dot(v1,v2);
       ang_sin = sqrt(1-ang_cos^2);
       l = r/ang_sin;
       A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;   
   end
end

%�ڶ���for����ֵ
for i= 1:num_sur
    B{i} = A{i};
end

%������for:����������淨����
for i=1:num_sur
    Vz(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %�������η�����,�����ռ�����ϵ
    Vz_norm=norm(Vz(i,:));
    Vz(i,:)=Vz(i,:)/Vz_norm;
end

%���ĸ�for�����ÿ�������y�᷽��x��Ϊ��һ���ߣ�z��Ϊ�������������ñ�������ϵ
for i = 1:num_sur
    Vx(i,:) = A{i}(2,:)-A{i}(1,:);
    Vx_norm=norm(Vx(i,:));
    Vx(i,:)=Vx(i,:)/Vx_norm;
    Vy(i,:)=cross(Vz(i,:),Vx(i,:));
end

%�����for������ÿ���������ת����
%R_swƽ������ϵ����������ϵ�µ�ת������
%R_ws��������ϵ��ƽ������ϵ�µ�ת������
for i =1:num_sur
R_sw(:,:,i)=[dot(Vx(i,:),Xa),dot(Vy(i,:),Xa),dot(Vz(i,:),Xa);dot(Vx(i,:),Ya),dot(Vy(i,:),Ya),dot(Vz(i,:),Ya);dot(Vx(i,:),Za),dot(Vy(i,:),Za),dot(Vz(i,:),Za)]; 
R_ws(:,:,i) = R_sw(:,:,i)';
end

%������for��ȷ��ÿ�������ԭ��
for i=1:num_sur
    pw(i,:) = A{i}(1,:);%ÿ�������ԭ�㶼��ÿ������ĵ�һ����
    psw(i,:) =(-R_ws(:,:,i)*pw(i,:)')';  %��������ϵ��ԭ����ƽ������ϵ�ϵı�ʾ
end

%���߸�for����ÿ������Ķ��㶼ת����ƽ������ϵ��
for i=1:num_sur
    for j=1:num_point(i)
        B{i}(j,:) = (R_ws(:,:,i)*A{i}(j,:)')'+psw(i,:);
    end
end

x=[ Pstart;
     x
    Pend];

%   x=[3.9499    7.1959    2.0064    
%     4.3380    7.2582    2.7340];%������Ҫ����
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

%for i=1:2 %mark�������Ҳ�֪��2�Ǵ���ʲô������˵Ӧ���Ǳ��������ŶԵģ��������ȸĳ�num_sur����
se_footstep3d = {}
% for i=1:num_sur
for i=1:num_sur
    ps=x(2*i-1,:);%��i������Ŀ�ʼ��
    pe=x(2*i,:);%��i��������յ�
    ps=(R_ws(:,:,i)*ps')'+psw(i,:);
    pe=(R_ws(:,:,i)*pe')'+psw(i,:);
    map_border=zeros(num_point(i),2);
    for j=1:num_point(i)
        map_border(j,:) = B{i}(j,1:2);%��ƽ������ϵ�£����ֻ��ȡ��������
    end
    % ���ص����������Ӧ���ǰ���ʼĩ���
    [ se_footstep3d{1,i},numst ] = footstepsolution( ps(1,1:2),pe(1,1:2),map_border,Obs_border{i},Obs_height{i});
    nums = numst +1;%������������ʼ���ȫ���������-1��
    step_op(num_op+1:num_op+nums-2) = 1;
    step_op(num_op+nums-1:num_op+nums) = 2;%�����յ㣬��������㶼�Ǳ�2��
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

