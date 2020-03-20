function [  ] = jointangle_transform( JJ )
%将关节角数据转换成攀爬机器人离线运动代码
fid = fopen('data.txt','w');
num_st = size(JJ,2);
num_p = 0;
ba_joi = zeros(1,5);
ba_joi = [0,8.463,-16.927,8.463,0];
cha_joi = zeros(1,5);
for i = 1:num_st
    num_j = size(JJ{i},1);
    if mod( i,2) ==1
        for j = 1:num_j-1
            cha_joi = JJ{i}(j+1,:)-JJ{i}(j,:);
            ba_joi = ba_joi +cha_joi;
             fprintf(fid,'P%d=',j+num_p);
              %fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',JJ{i}(j,1),JJ{i}(j,2),JJ{i}(j,3),JJ{i}(j,4),JJ{i}(j,5));
              fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
              fprintf(fid,'\r\n');  
              
        end
      %  ba_joi =[ba_joi(5),ba_joi(4),ba_joi(3),ba_joi(2),ba_joi(1)];
    else
        for j = 1:num_j-1
             cha_joi = JJ{i}(j+1,:)-JJ{i}(j,:);
            ba_joi = ba_joi +[cha_joi(5),cha_joi(4),cha_joi(3),cha_joi(2),cha_joi(1)];
             fprintf(fid,'P%d=',j+num_p);
             % fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',JJ{i}(j,5),JJ{i}(j,4),JJ{i}(j,3),JJ{i}(j,2),JJ{i}(j,1));
              fprintf(fid,'%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,;',ba_joi(1,1),ba_joi(1,2),ba_joi(1,3),ba_joi(1,4),ba_joi(1,5));
              fprintf(fid,'\r\n');  
        end
    end
    num_p = num_p +num_j-1;
    step_inter(i) = num_p;
end
for i = 1:num_p
     fprintf(fid,'MOVJ P%d, V70, Z1;',i);
      fprintf(fid,'\r\n'); 
      for j = 1:num_st-1
      if i == step_inter(j)
          fprintf(fid,'\r\n');
      end
      end
end
end