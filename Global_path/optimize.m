function [f,ceq] = optimize(x) %x����������֮��Ĺ��ɵ㣨��P1^B,P1^E����
global L1 L2;
% L1= 0.3407;L2=0.293;
% L1 = 0.335;
[A1,num_point,num_sur,Pstart,Pend] = surfaces;
r = 0.2;
Y=[0,1,0];
Vn=zeros(num_sur,3);
Pc=zeros(num_sur,3);
Vz=zeros((num_sur-1)*2,3);
Vy=zeros((num_sur-1)*2,3);
Vx=zeros((num_sur-1)*2,3);
angle_sum=zeros(num_sur-1,1);
Ptr=zeros((num_sur-1)*2,3);
angle=zeros(num_sur-1,3);
Ptar=zeros(num_sur-1,3);
B= cell((num_sur-1)*2,1);        %����ε��������ϵת����Ĵ���
Vn2=zeros((num_sur-1)*2,3);       %�����ת����ķ�����
Pc2=zeros((num_sur-1)*2,3);       %�����ת��������ĵ�
lin = cell(num_sur-1,1);        %���������
jud = zeros((num_sur-1)*2,3);    %�����������ཻ�ж�ϵ��
num_ceq=0;num_f=0;
%[  Vn,Pc,Vz,Vy,Vx,angle_sum ] = opti_data;


%///////////////����߽�����
for i=1:num_sur
     %��һ�飨1�����������ĳ�����ڱ߷ֱ����ڵ�ֱ���ϵĵ�λ������2�������ŵ�ԭ���ڱ߽��㳯��λ����֮�͵ķ��������ƶ�
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
   for j=2:num_point(i)-1
       v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
       v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
       ang_cos = dot(v1,v2);
       ang_sin = sqrt(1-ang_cos^2);
       l = r/ang_sin;
       A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;
   end
end

%///////////////�������η����������б���ĵ�λ��������
for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %�������η����������ĵ�
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm;
end

%//////////////�����������ĵ㣨���б�������ĵ㣩
for i=1:num_sur
    for k = 1:num_point(i)
        Pc(i,:)= Pc(i,:)+A{i}(k,:);
    end
   Pc(i,:) = Pc(i,:)/num_point(i);
end

%{
%/////////////////////////////////
%������������ճ�һ�飬���Խ�������ϵ���Լ���������������ת��
%���У���������ϵ��y���Ƿ�����ͬ�ģ�z���Ǳ��淨��������ÿ��ĵڶ�������ȡ���������෴���򣨿�����Ϊ�˷�������̬����
%}
for i=1:num_sur-1                                               %�����������֮�������ϵ��λ��������
    Vz(2*i-1,:)=Vn(i,:);                                
    Vz(2*i,:)=Vn(i+1,:)*-1; 
end
for i=1:num_sur-1
    Vy(2*i-1,:)=cross(Vz(2*i-1,:),Vz(2*i,:));
    if dot(Vy(2*i-1,:),Y)<0
    Vy(2*i-1,:)= -Vy(2*i-1,:);
    end
    %�����ϣ���ʱ��Vy�Ѿ��ǵ�λ�����ˣ������ڲ�˵ļ�����ʹVy�����һ���ܽӽ�1����������0.9991������������Ҫ���±�׼��
    Vy_norm=norm(Vy(2*i-1,:));
    Vy(2*i-1,:)=Vy(2*i-1,:)/Vy_norm;
    Vy(2*i,:) =  Vy(2*i-1,:);
end
for i= 1:num_sur-1
    %�����������֮���ת��
    %��ʵ�ϣ����ǽ���������ϵ��y���غϷ�һ�������z��ļн�
    Vx(2*i-1,:)=cross(Vy(2*i-1,:),Vz(2*i-1,:));
    Vx(2*i,:)=cross(Vy(2*i,:),Vz(2*i,:));
end

%////////////////����ƽ�ƺ�ĵ��λ�� 
for i= 1:num_sur-1 %�����������֮���ת�� 
   xx=dot(Vx(2*i,:),Vx(2*i-1,:));
   yy=dot(Vx(2*i,:),Vz(2*i-1,:));
   angle_sum(i)=atan2(yy,xx);
end
for i= 1:num_sur-1                            %����ƽ�ƺ�ĵ��λ�� 
   Ptr(2*i-1,:) = x(2*i-1,:)+Vn(i,:)*L1;        
   Ptr(2*i,:) = x(2*i,:)+Vn(i+1,:)*L1;
end
for i= 1:num_sur-1                           
  %����Ŀ���������ڻ�����ϵ�µı�ʾ
  %Q����˼�ǣ�2*i-1ָ���ǵ�ǰ���ڵı��棬��2*i����һ��Ҫ����ı��棿A���ǵ�
  Ptar(i,:) = [dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vx(2*i-1,:)), dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vy(2*i-1,:)), dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vz(2*i-1,:))];
end

%Q�������������˶�ѧ������ת�ǣ���������������������ת�ǣ�����������˵����˶�ѧ�Ƶ��������
 for i= 1:num_sur-1
     angle(i,1) = acos((Ptar(i,1)^2+Ptar(i,3)^2)/(2*L2*sqrt(Ptar(i,1)^2+Ptar(i,3)^2)))+atan2(Ptar(i,3),Ptar(i,1));    %�������˶�ѧ������ת��
     angle(i,2) = -acos((Ptar(i,1)^2+Ptar(i,3)^2-2*L2^2)/(2*L2^2));
     angle(i,3) = angle_sum(i) - angle(i,2) - angle(i,1) +1.57;  
 end
for i=1:num_sur-1
    ceq(2*i-1)=dot(x(2*i-1,:)-Pc(i,:),Vn(i,:));                              %��ʽԼ�������ڶ��������ƽ����   
    ceq(2*i)=dot(x(2*i,:)-Pc(i+1,:),Vn(i+1,:));
end
num_ceq=num_ceq+ 2*(num_sur-1);
for i=1:num_sur-1
    for k=1:num_point(i)
        if k ==num_point(i)
            f(num_f+k)=-dot(cross(x(2*i-1,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)));
        else 
            f(num_f+k)=-dot(cross(x(2*i-1,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)));            %����ʽԼ�������ڶ���α߽���
        end
    end
num_f = num_f+num_point(i);
end
for i=2:num_sur
    for k= 1:num_point(i)
        if k==num_point(i)
          f(num_f+k)=-dot(cross(x(2*i-2,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(1,:)-A{i}(k,:)));
          else 
            f(num_f+k)=-dot(cross(x(2*i-2,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)),cross(Pc(i,:)-A{i}(k,:),A{i}(k+1,:)-A{i}(k,:)));            %����ʽԼ�������ڶ���α߽���
        end
    end
    num_f = num_f+num_point(i);
end
for i=1:num_sur-1
    f(num_f+1)=-sin(angle(i,1))-0.866;                             %����ת�ǲ���ʽԼ��,��ת�ؽ�30��Լ��
   f(num_f+2)=-cos(angle(i,2))-0.5;
   f(num_f+3)=-cos(angle(i,3))-0.5;
   num_f = num_f+3;
end
for i=1:num_sur-1
%     f(num_f+i)=norm(Ptar(i,:))-0.586+0.002;             %��������������֮���λ��Լ�� %2020.1.12ע��
    f(num_f+i)=norm(Ptar(i,:))-0.5+0.002;             %��������������֮���λ��Լ��
    ceq(num_ceq+i)=dot(Ptr(2*i,:)-Ptr(2*i-1,:),Vy(2*i-1,:));
end
num_f = num_f + num_sur-1;
num_ceq = num_ceq + num_sur-1;
for i=1:num_sur-1                                    %��ת����ĵ������ʼ��
    B{2*i-1}=zeros(size(A{i},1),size(A{i},2));
    B{2*i}=zeros(size(A{i+1},1),size(A{i+1},2));
end
for i=1:num_sur-1                            %������ת����ĵ�����
    for k=1:num_point(i)
        B{2*i-1}(k,:) = [dot(A1{i}(k,:)-x(2*i-1,:),Vx(2*i-1,:)),dot(A1{i}(k,:)-x(2*i-1,:),Vy(2*i-1,:)),dot(A1{i}(k,:)-x(2*i-1,:),Vz(2*i-1,:))];
    end
    for k=1:num_point(i+1)
         B{2*i}(k,:) = [dot(A1{i+1}(k,:)-x(2*i-1,:),Vx(2*i-1,:)),dot(A1{i+1}(k,:)-x(2*i-1,:),Vy(2*i-1,:)),dot(A1{i+1}(k,:)-x(2*i-1,:),Vz(2*i-1,:))];
    end
end

for i=1:num_sur-1                     %������ת����Ķ���εķ����������ĵ�
     Vn2(2*i-1,:)=cross(B{2*i-1}(2,:)-B{2*i-1}(1,:),B{2*i-1}(3,:)-B{2*i-1}(2,:));  
    Vn_norm=norm(Vn2(2*i-1,:));
    Vn2(2*i-1,:)=Vn2(2*i-1,:)/Vn_norm;
     Vn2(2*i,:)=cross(B{2*i}(2,:)-B{2*i}(1,:),B{2*i}(3,:)-B{2*i}(2,:));  
    Vn_norm=norm(Vn2(2*i,:));
    Vn2(2*i,:)=Vn2(2*i,:)/Vn_norm;
end
for i=1:num_sur-1
    for k = 1:num_point(i)
        Pc2(2*i-1,:)= Pc2(2*i-1,:)+B{2*i-1}(k,:);
    end
   Pc2(2*i-1,:) = Pc2(2*i-1,:)/num_point(i);
    for k = 1:num_point(i+1)
        Pc2(2*i,:)= Pc2(2*i,:)+B{2*i}(k,:);
    end
   Pc2(2*i,:) = Pc2(2*i,:)/num_point(i+1);
end
for i=1:num_sur-1
    lin{i}= zeros(5,3);
end
 for i=1:num_sur-1                                    %���������������
      lin{i}(1,:) = [0 0 0];
      lin{i}(2,:) = [0 0 L1];
      lin{i}(3,:) = [L2*cos(angle(i,1)),0, L1+L2*sin(angle(i,1))];
      lin{i}(4,:) = [lin{i}(3,1)+L2*cos(angle(i,1)+angle(i,2)),0,lin{i}(3,3)+L2*sin(angle(i,1)+angle(i,2))];
      lin{i}(5,:) = [lin{i}(4,1)+L1*cos(angle(i,1)+angle(i,2)+angle(i,3)),0,lin{i}(4,3)+L1*sin(angle(i,1)+angle(i,2)+angle(i,3))];
 end
for i= 1:num_sur-1
    for k= 2:4
       vec_link = lin{i}(k+1,:)-lin{i}(k,:);
       t_coe = dot(Vn2(2*i-1,:),B{2*i-1}(1,:)-lin{i}(k,:))/dot(Vn2(2*i-1,:),vec_link);
       inter_sec = lin{i}(k,:) + t_coe*vec_link;                                      %�������߶�������ƽ�潻��
       if t_coe>1||t_coe<0
           jud(2*i-1,k-1) = num_point(i);
       else
           for j=1:num_point(i)
            if j ==num_point(i)
           num_jud = dot(cross(inter_sec-B{2*i-1}(j,:),B{2*i-1}(1,:)-B{2*i-1}(j,:)),cross(Pc2(2*i-1,:)-B{2*i-1}(j,:),B{2*i-1}(1,:)-B{2*i-1}(j,:)));
        else 
            num_jud = dot(cross(inter_sec-B{2*i-1}(j,:),B{2*i-1}(j+1,:)-B{2*i-1}(j,:)),cross(Pc2(2*i-1,:)-B{2*i-1}(j,:),B{2*i-1}(j+1,:)-B{2*i-1}(j,:)));            %�㲻�ڶ���α߽���
            end 
            if num_jud <0
              jud(2*i-1,k-1)= jud(2*i-1,k-1)+1;
            end
           end
       end
    end
    for k = 1:3
      vec_link = lin{i}(k+1,:)-lin{i}(k,:);
       t_coe = dot(Vn2(2*i,:),B{2*i}(1,:)-lin{i}(k,:))/dot(Vn2(2*i,:),vec_link);
       inter_sec = lin{i}(k,:) + t_coe*vec_link;                                  %�������߶�������ƽ�潻��
       if t_coe>1||t_coe<0
           jud(2*i,k) = num_point(i+1);
       else
           for j=1:num_point(i+1)
            if j ==num_point(i+1)
           num_jud = dot(cross(inter_sec-B{2*i}(j,:),B{2*i}(1,:)-B{2*i}(j,:)),cross(Pc2(2*i,:)-B{2*i}(j,:),B{2*i}(1,:)-B{2*i}(j,:)));
        else 
            num_jud = dot(cross(inter_sec-B{2*i}(j,:),B{2*i}(j+1,:)-B{2*i}(j,:)),cross(Pc2(2*i,:)-B{2*i}(j,:),B{2*i}(j+1,:)-B{2*i}(j,:)));            %�㲻�ڶ���α߽���
            end 
            if num_jud <0
              jud(2*i,k)= jud(2*i,k)+1;
            end
           end
       end
    end   
end
 for i= 1:(num_sur-1)*2
    f(num_f+1)=-jud(i,1)+1;
    f(num_f+2)=-jud(i,2)+1;
    f(num_f+3)=-jud(i,3)+1;
     num_f=num_f+3;
 end
end
