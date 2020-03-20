clear
[A,num_point,num_sur] = surfaces;
L1= 0.3407;
pup=ones(num_sur*2-2,3)*10;
pzero=zeros(num_sur*2-2,3);
pinit = zeros(num_sur*2-2,3);
prand = rand*[0.2,0.1,0.2];
Vn=zeros(num_sur,3);
for i=1:num_sur
    Vn(i,:)=cross(A{i}(2,:)-A{i}(1,:),A{i}(3,:)-A{i}(2,:));   %计算多边形单位法向量
    Vn_norm=norm(Vn(i,:));
    Vn(i,:)=Vn(i,:)/Vn_norm;
end
for i=1:num_sur-1
    n_num = ceil(rand*num_point(i));
    if n_num==num_point(i)
        n_num2 = 1;
    else
        n_num2 = n_num +1;
    end
    pinit(2*i-1,:) = A{i}(n_num,:)+(A{i}(n_num2,:)-A{i}(n_num,:))*rand;
end
    
for i=1:num_sur-1
    pinit(2*i-1,:) = pinit(2*i-1,:) + Vn(i,:)*L1;
    %pinit(2*i-1,:) =  rand(1,3)*9;
    pinit(2*i,:) = pinit(2*i-1,:)+ [0.2,0,0.2];
end

for i=1:num_sur-1
    pinit(2*i-1,:) =  pinit(2*i-1,:) - Vn(i,:)*L1;
    pinit(2*i,:) = pinit(2*i,:) - Vn(i+1,:)*L1;
end

    options = optimset('MaxFunEvals',7000);
    %pinit=[2,1,3;2,1,3.3;3,1,4;3,1,4.3;4,1,2;4,1,2.3;3,2,2;3,2,2.3];
tic
[x,fval] = fmincon('min_dis',pinit,[],[],[],[],pzero,pup,'optimize',options)
toc
%disp(['运行时间: ',num2str(toc)]);