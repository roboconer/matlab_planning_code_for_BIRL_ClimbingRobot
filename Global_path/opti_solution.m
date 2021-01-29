% clear
% [A,num_point,num_sur,~,~] = surfaces;
% r=0.2;
% global L1 L2;
% L1 = 0.335;
% L2 = 0.2932;
% pup=ones(num_sur*2-2,3)*3;       %取值范围上限
% pzero=zeros(num_sur*2-2,3)-2;       %取值范围下限
% pinit = zeros(num_sur*2-2,3);
% prand = rand*[0.2,0.1,0.2];
% Vn=zeros(num_sur,3);
% 
% %计算每个壁面的法向量
% for i=1:num_sur
%     Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形单位法向量
%     Vn_norm=norm(Vn(i,:));
%     Vn(i,:)=Vn(i,:)/Vn_norm;
% end
% 
% %这里产生初值的思路看得不大明白，要问问师兄
% for i=1:num_sur-1
%     n_num = ceil(rand*num_point(i));
%     if n_num==num_point(i)
%         n_num2 = 1;
%     else n_num2 = n_num +1;
%     end
%     pinit(2*i-1,:) = A{i}(n_num,:)+(A{i}(n_num2,:)-A{i}(n_num,:))*rand;
% end
%     
% for i=1:num_sur-1
% %     pinit(2*i-1,:) = pinit(2*i-1,:) + Vn(i,:)*L1;
%       pinit(2*i-1,:) =  rand(1,3)*3-[0.5 0.5 0.5];
%      %  pinit(2*i-1,:) =  A{i}(1,:);
%     pinit(2*i,:) = pinit(2*i-1,:)+ [0.2,0.2,0.2];
% end
% 
% for i=1:num_sur-1
%     pinit(2*i-1,:) =  pinit(2*i-1,:) - Vn(i,:)*L1;
%     pinit(2*i,:) = pinit(2*i,:) - Vn(i+1,:)*L1;
% end
% %pinit=[6.3945,0.4142,4.4715;6.3565,0.2008,5.1072;5.057,2.7208,4.1145;4.7571,2.0826,3.5339;1.8687,1.7688,6.3974;1.4523,2.5293,6.9486;6.3957,2.9085,4.6701;6.9675,3.0714,5.1319];
% x=[5.0570    2.7208    4.1145
%     4.7571    2.0826    3.5339
%     1.8687    1.7688    6.3974
%     1.4523    2.5293    6.9486
%     2.3496    5.2621    9.2806
%     3.2442    5.1693    9.7999];
% 
% options = optimset('MaxFunEvals',5000);
% tic
% [x,fval] = fmincon('min_dis',pinit,[],[],[],[],pzero,pup,'optimize',options)
% toc
%   fprintf('pinit=[');
% for i = 1:num_sur-1
%     fprintf('%3.4f,%3.4f,%3.4f;',pinit(2*i-1,1),pinit(2*i-1,2),pinit(2*i-1,3));
%     fprintf('%3.4f,%3.4f,%3.4f;',pinit(2*i,1),pinit(2*i,2),pinit(2*i,3));
%    
% end
%     fprintf('];');
%     fprintf('\n');
% %disp(['运行时间: ',num2str(toc)]);


clear
[A,num_point,num_sur,~,~] = surfaces;
r=0.2;
global L1 L2;
%L1= 0.3407;
L1 = 0.329;
L2 = 0.2932;
pup=ones(num_sur*2-2,3)*4;       %取值范围上限
pzero=zeros(num_sur*2-2,3)-4;       %取值范围下限
pinit = zeros(num_sur*2-2,3);
prand = rand*[0.2,0.1,0.2];
Vn=zeros(num_sur,3);
% for i=1:num_sur
%     v1= (A1{i}(num_point(i),:)-A1{i}(1,:))/norm(A1{i}(num_point(i),:)-A1{i}(1,:));
%     v2= (A1{i}(2,:)-A1{i}(1,:))/norm(A1{i}(2,:)-A1{i}(1,:));
%     ang_cos = dot(v1,v2);
%     ang_sin = sqrt(1-ang_cos^2);
%     l = r/ang_sin;
%     A{i}(1,:) = A1{i}(1,:)+ v1*l +v2*l;
%     
%     v1=( A1{i}(1,:)-A1{i}(num_point(i),:))/norm(A1{i}(1,:)-A1{i}(num_point(i),:));
%     v2= (A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:))/norm(A1{i}(num_point(i)-1,:)-A1{i}(num_point(i),:));
%     ang_cos = dot(v1,v2);
%     ang_sin = sqrt(1-ang_cos^2);
%     l = r/ang_sin;
%     A{i}(num_point(i),:) = A1{i}(num_point(i),:)+ v1*l +v2*l;
%     
%    for j=2:num_point(i)-1
%        v1= (A1{i}(j+1,:)-A1{i}(j,:))/norm(A1{i}(j+1,:)-A1{i}(j,:));
%        v2= (A1{i}(j-1,:)-A1{i}(j,:))/norm(A1{i}(j-1,:)-A1{i}(j,:));
%        ang_cos = dot(v1,v2);
%        ang_sin = sqrt(1-ang_cos^2);
%        l = r/ang_sin;
%        A{i}(j,:) = A1{i}(j,:)+ v1*l +v2*l;
%        
%    end
% end
for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形单位法向量
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm;
end

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

%%%%%%%%%%%%%%%%%%%%%%%%%%设置初始点在壁面上
for i = 1:num_sur-1
   pinit(2*i-1,:) = A{i}(1,:) +(A{i}(3,:) - A{i}(2,:))*0.2 + (A{i}(2,:) - A{i}(1,:))*0.5 + Vn(i,:)*0.1;
   pinit(2*i,:) = A{i+1}(1,:) +(A{i+1}(3,:) - A{i+1}(2,:))*0.3 + (A{i+1}(2,:) - A{i+1}(1,:))*0.2 + Vn(i+1,:)*0.1;
end


%pinit=[6.3945,0.4142,4.4715;6.3565,0.2008,5.1072;5.057,2.7208,4.1145;4.7571,2.0826,3.5339;1.8687,1.7688,6.3974;1.4523,2.5293,6.9486;6.3957,2.9085,4.6701;6.9675,3.0714,5.1319];
% x = [5.0570    2.7208    4.1145
%     4.7571    2.0826    3.5339
%     1.8687    1.7688    6.3974
%     1.4523    2.5293    6.9486
%     2.3496    5.2621    9.2806
%     3.2442    5.1693    9.7999];
x = [];
    
    
% pinit(1:8,:)=[    4.5492    6.6332    1.7340
%     4.8766    6.6961    2.3305
%     7.0142    6.9121    5.4006
%     6.5623    6.1258    6.2337
%     4.7940    2.7199    4.0578
%     4.5002    2.0818    3.7416
%     1.9462    1.9499    6.0207
%     1.3683    2.6175    6.5359
%    ];
%pinit=[6.9189,3.5160,0.2860;7.6888,3.0998,0.9908;7.7561,3.6706,8.5222;7.2260,3.9669,9.3559;3.7719,5.8361,4.7416;3.8351,5.7236,4.7223;7.4990,3.2724,1.0601;7.7578,3.9032,0.8270;];



% pinit(1:6,:)=[0.6387    0.3270    0.9001
%    -0.1535    1.0210    0.9001
%    -0.1535    1.0210    0.9001
%     0.0054    1.6813    0.9001
%     0.0054    1.6813    0.9001
%     0.7076    2.0397    0.9001];

% 
% pinit=[     
%     -0.0000    1.0000    1.5452
%     0.3216    1.0000    2.3216
%    ];

options = optimset('MaxFunEvals',1000);
tic
[x,fval,exit_flag] = fmincon('min_dis',pinit,[],[],[],[],pzero,pup,'optimize',options)
toc
  fprintf('pinit=[');
for i = 1:num_sur-1
    fprintf('%3.4f,%3.4f,%3.4f;',pinit(2*i-1,1),pinit(2*i-1,2),pinit(2*i-1,3));
    fprintf('%3.4f,%3.4f,%3.4f;',pinit(2*i,1),pinit(2*i,2),pinit(2*i,3));
   
end
    fprintf('];');
    fprintf('\n');
%disp(['运行时间: ',num2str(toc)]);