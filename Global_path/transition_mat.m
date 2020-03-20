function [ M_adj ] = transition_mat(  )
%求壁面可过渡性邻接矩阵
L1= 0.340;L2=0.293;Y=[0,1,0];
[A1,num_point,num_sur,~,~] = input_surfaces;
M_adj = zeros(num_sur,num_sur);
r = 0.2;
Vn=zeros(num_sur,3);
A= A1;
tic
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
for i = 1:num_sur
for j = 1:num_point(i)
    Aup{i}(j,:) = A{i}(j,:)+ Vn(i,:)*L1;
end
end
R_c = 2*L2;
for i = 1:num_sur
    for j = i + 1: num_sur
        c_vec = cross(Vn(i,:),Vn(j,:));
        state = trans_ab(Aup{i},Aup{j},c_vec,R_c);
        if state ==1
        M_adj(i,j) = 1;
        M_adj(j,i) = 1;
        elseif state == 0
            M_adj(i,j) = inf;
            M_adj(j,i) = inf;
        end
    end
end
toc
end
%%
function state = trans_ab(poly1,poly2,c_vec,R_c) %求机器人能否在两个壁面间过渡
num_p1 = size(poly1,1);
num_p2 = size(poly2,1);
for i = 1:num_p1
if i == num_p1
    l1 = [poly1(num_p1,:);poly1(1,:)];
else
    l1 = [poly1(i,:);poly1(i+1,:)];
end
for j = 1:num_p2
    if j == num_p2
    l2 = [poly2(num_p2,:);poly2(1,:)];
    else
    l2 = [poly2(j,:);poly2(j+1,:)];
    end
    [dist,lp1,lp2] = Seg2SegDist(l1,l2);
    state1 = circlepoly_inter( lp1,R_c,c_vec,poly2 );
    state2 = circlepoly_inter( lp2,R_c,c_vec,poly1 );
    if state1 ==1 || state2 == 1
        state = 1;
        return;
    end
end
end
state = 0;
end







