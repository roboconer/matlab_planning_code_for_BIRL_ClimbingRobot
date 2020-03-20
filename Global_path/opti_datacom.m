function [A] = opti_datacom( )
%计算优化算法的必要数据
%   此处显示详细说明
L1= 0.3407;L2=0.29325;Y=[0,1,0];
[A1,num_point,num_sur,~,~] = surfaces;
r = 0.25;
Vn=zeros(num_sur,3);
Pc=zeros(num_sur,3);
Vz=zeros((num_sur-1)*2,3);
Vy=zeros((num_sur-1)*2,3);
Vx=zeros((num_sur-1)*2,3);
angle_sum=zeros(num_sur-1,1);
Ptr=zeros((num_sur-1)*2,3);
angle=zeros(num_sur-1,3);
Ptar=zeros(num_sur-1,3);
B= cell((num_sur-1)*2,1);        %多边形点进行坐标系转换后的储存
Vn2=zeros((num_sur-1)*2,3);       %多边形转换后的法向量
Pc2=zeros((num_sur-1)*2,3);       %多边形转换后的中心点
lin = cell(num_sur-1,1);        %连杆坐标点
jud = zeros((num_sur-1)*2,3);    %连杆与多边形相交判断系数
num_ceq=0;num_f=0;
for i=1:num_sur
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
for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形法向量和中心点
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm;
end
for i=1:num_sur
    for k = 1:num_point(i)
        Pc(i,:)= Pc(i,:)+A{i}(k,:);
    end
   Pc(i,:) = Pc(i,:)/num_point(i);
end
for i=1:num_sur-1                                               %计算两多边形之间的坐标系单位方向向量
    Vz(2*i-1,:)=Vn(i,:);                                
    Vz(2*i,:)=Vn(i+1,:)*-1; 
end
for i=1:num_sur-1
    Vy(2*i-1,:)=cross(Vz(2*i-1,:),Vz(2*i,:));
    if dot(Vy(2*i-1,:),Y)<0
    Vy(2*i-1,:)= -Vy(2*i-1,:);
    end
    Vy_norm=norm(Vy(2*i-1,:));
    Vy(2*i-1,:)=Vy(2*i-1,:)/Vy_norm;
    Vy(2*i,:) =  Vy(2*i-1,:);
end
for i= 1:num_sur-1
    Vx(2*i-1,:)=cross(Vy(2*i-1,:),Vz(2*i-1,:));
    Vx(2*i,:)=cross(Vy(2*i,:),Vz(2*i,:));
end
for i= 1:num_sur-1                            %计算两多边形之间的转角 
   xx=dot(Vx(2*i,:),Vx(2*i-1,:));
   yy=dot(Vx(2*i,:),Vz(2*i-1,:));
   angle_sum(i)=atan2(yy,xx);
end


 fprintf('Vn=[');
for i=1:num_sur
   
    fprintf('%3.4f,%3.4f,%3.4f;',Vn(i,1),Vn(i,2),Vn(i,3));
    
end
fprintf('];');
    fprintf('\n');
     fprintf('Pc=[');
for i =1:num_sur
    fprintf('%3.4f,%3.4f,%3.4f;',Pc(i,1),Pc(i,2),Pc(i,3));
end
 fprintf('];');
    fprintf('\n');
     fprintf('Vx=[');
for i = 1:num_sur-1
    fprintf('%3.4f,%3.4f,%3.4f;',Vx(2*i-1,1),Vx(2*i-1,2),Vx(2*i-1,3));
    fprintf('%3.4f,%3.4f,%3.4f;',Vx(2*i,1),Vx(2*i,2),Vx(2*i,3));
   
end
    fprintf('];');
    fprintf('\n');
         fprintf('Vy=[');
for i = 1:num_sur-1
    fprintf('%3.4f,%3.4f,%3.4f;',Vy(2*i-1,1),Vy(2*i-1,2),Vy(2*i-1,3));
    fprintf('%3.4f,%3.4f,%3.4f;',Vy(2*i,1),Vy(2*i,2),Vy(2*i,3));
   
end
    fprintf('];');
    fprintf('\n');  
    fprintf('Vz=[');
for i = 1:num_sur-1
    fprintf('%3.4f,%3.4f,%3.4f;',Vz(2*i-1,1),Vz(2*i-1,2),Vz(2*i-1,3));
    fprintf('%3.4f,%3.4f,%3.4f;',Vz(2*i,1),Vz(2*i,2),Vz(2*i,3));
   
end
    fprintf('];');
    fprintf('\n');
     fprintf('angle_sum=[');
     for i=1:num_sur-1
        fprintf('%3.4f,',angle_sum(i)); 
     end
      fprintf('];');
      fprintf('\n');  
      num_sur2=size(A,2);

num_point=zeros(num_sur2,1);
for i=1:num_sur2
    num_point(i)=size(A{i},1);
end
for i=1:num_sur2
    fprintf('A{%d}=[',i);
    for k=1:num_point(i)
        if k == num_point(i)
             fprintf('%3.4f,%3.4f,%3.4f',A{i}(k,1),A{i}(k,2),A{i}(k,3));
        else
         fprintf('%3.4f,%3.4f,%3.4f;',A{i}(k,1),A{i}(k,2),A{i}(k,3));
        end
    end
    %fprintf('%3.4f,%3.4f,%3.4f; ',A{i}(1,:),A{i}(2,:),A{i}(3,:));
    fprintf('];');
     fprintf('\n');
end
end

