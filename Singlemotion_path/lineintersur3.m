function [ po ] = lineintersur3(line,poly )
%��ֱ����ƽ��Ľ���
 vn=cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));   %�������η����������ĵ�
 
  vv = line(2,:)-line(1,:);
  if abs(dot(vv,vn))<1e-4
       state = 0;
       po=0;
       return;
  end
   t = dot(vn,poly(1,:)-line(1,:))/dot(vn,vv);
  
       state = 1;
       po = line(1,:)+ vv*t;
   

end

