function [] = surface_data()
A = opti_datacom( );
num_sur2=size(A,2);
% for i= 1:num_sur2
%     A{i} = A{i}/1000;
% end
num_point=zeros(num_sur2,1);
for i=1:num_sur2
    num_point(i)=size(A{i},1);
end
for i=1:num_sur2
    fprintf('Ps%d=[',i);
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
for i=1:num_sur2
    for k=1:num_point(i)
        if k == num_point(i)&&i==num_sur2
            fprintf('s%d%d(%3.4f,%3.4f,%3.4f);',i,k,A{i}(k,:));
        else
          fprintf('s%d%d(%3.4f,%3.4f,%3.4f),',i,k,A{i}(k,:));
        end
    end
%fprintf('s%d1(%3.4f,%3.4f,%3.4f),s%d2(%3.4f,%3.4f,%3.4f),s%d3(%3.4f,%3.4f,%3.4f),',i,A{i}(1,:),i,A{i}(2,:),i,A{i}(3,:));
end
 fprintf('\n');
end