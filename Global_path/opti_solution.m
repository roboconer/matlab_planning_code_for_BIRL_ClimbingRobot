clear
[A,num_point,num_sur,~,~] = surfaces;
r=0.2;
global L1 L2;
L1 = 0.335;
L2 = 0.2932;
pup=ones(num_sur*2-2,3)*3;       %取值范围上限
pzero=zeros(num_sur*2-2,3)-2;       %取值范围下限
pinit = zeros(num_sur*2-2,3);
prand = rand*[0.2,0.1,0.2];
Vn=zeros(num_sur,3);

%计算每个壁面的法向量
for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形单位法向量
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm;
end

%这里产生初值的思路看得不大明白，要问问师兄
for i=1:num_sur-1
    n_num = ceil(rand*num_point(i));
    if n_num==num_point(i)
        n_num2 = 1;
    else n_num2 = n_num +1;
    end
    pinit(2*i-1,:) = A{i}(n_num,:)+(A{i}(n_num2,:)-A{i}(n_num,:))*rand;
end
    
for i=1:num_sur-1
%     pinit(2*i-1,:) = pinit(2*i-1,:) + Vn(i,:)*L1;
      pinit(2*i-1,:) =  rand(1,3)*3-[0.5 0.5 0.5];
     %  pinit(2*i-1,:) =  A{i}(1,:);
    pinit(2*i,:) = pinit(2*i-1,:)+ [0.2,0.2,0.2];
end

for i=1:num_sur-1
    pinit(2*i-1,:) =  pinit(2*i-1,:) - Vn(i,:)*L1;
    pinit(2*i,:) = pinit(2*i,:) - Vn(i+1,:)*L1;
end
%pinit=[6.3945,0.4142,4.4715;6.3565,0.2008,5.1072;5.057,2.7208,4.1145;4.7571,2.0826,3.5339;1.8687,1.7688,6.3974;1.4523,2.5293,6.9486;6.3957,2.9085,4.6701;6.9675,3.0714,5.1319];
x=[5.0570    2.7208    4.1145
    4.7571    2.0826    3.5339
    1.8687    1.7688    6.3974
    1.4523    2.5293    6.9486
    2.3496    5.2621    9.2806
    3.2442    5.1693    9.7999];

options = optimset('MaxFunEvals',5000);
tic
[x,fval] = fmincon('min_dis',pinit,[],[],[],[],pzero,pup,'optimize',options)
toc
  fprintf('pinit=[');
for i = 1:num_sur-1
    fprintf('%3.4f,%3.4f,%3.4f;',pinit(2*i-1,1),pinit(2*i-1,2),pinit(2*i-1,3));
    fprintf('%3.4f,%3.4f,%3.4f;',pinit(2*i,1),pinit(2*i,2),pinit(2*i,3));
   
end
    fprintf('];');
    fprintf('\n');
%disp(['运行时间: ',num2str(toc)]);