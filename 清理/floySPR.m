%floydsrr算法
%a-赋权邻接矩阵
%D-距离矩阵
%R-最短路径矩阵
function [ D ,R ]=floySPR(a)
n=size(a,1);
D=a;
R=zeros(n,n);
for i=1:n;
    for j=1:n;
        if D(i,j)~=inf
            R(i,j)=j;
        end
    end
end
for k=1:n
    for i=1:n
        for j=1:n
            if D(i,k)+D(k,j)<D(i,j)
                D(i,j)=D(i,k)+D(k,j);
                R(i,j)=R(i,k);
            end
        end
    end
end
