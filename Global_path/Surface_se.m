function [  ] = Surface_se( num_sta,num_end )
%�����֪��ʼ��������ձ�������µ�ȫ�ֱ�������,���ȫ�ֱ���������ÿһ������ľ�������
[A2,~,~,Pstart,Pend] = input_surfaces;
[ M_adj ] = transition_mat();
% num_sta = 7;   %��ʼ�����Ŀ�����ı�ţ������
% num_end = 5;
possiablePaths = findPath_dsf(M_adj,num_sta,num_end,0);%��ȫ�ֱ������У����һ������Ĭ��Ϊ0
num_pa = size(possiablePaths,1); %ȫ��·����Ŀ
num_s1 = size(possiablePaths,2);
for i = 1:num_pa
num_sur2 = possiablePaths(i,num_s1)+1;
for j = 1:num_sur2
   A{j} = A2{(possiablePaths(i,j))}; 
end
end
fprintf('ȫ�ֱ���������%d��',num_pa);
fprintf('\n');
for i = 1:num_pa
fprintf('\n');
fprintf('��%d��:',i);
fprintf('\n');
num_sur2 = possiablePaths(i,num_s1)+1;
for j = 1:num_sur2
    fprintf('%d  ',possiablePaths(i,j));
     A{j} = A2{(possiablePaths(i,j))}; 
end
fprintf('\n');
for j = 1:num_sur2
 fprintf('A{%d}=[',j);
    for k=1:size(A{j},1)
        if k == size(A{j},1)
             fprintf('%3.4f,%3.4f,%3.4f',A{j}(k,1),A{j}(k,2),A{j}(k,3));
        else
         fprintf('%3.4f,%3.4f,%3.4f;',A{j}(k,1),A{j}(k,2),A{j}(k,3));
        end
    end
    %fprintf('%3.4f,%3.4f,%3.4f; ',A{i}(1,:),A{i}(2,:),A{i}(3,:));
    fprintf('];');
    fprintf('\n');
end
end
end

