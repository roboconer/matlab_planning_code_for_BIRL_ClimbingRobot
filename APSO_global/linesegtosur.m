function [state,po] = linesegtosur(line,o,cir_n)%�߶������󽻵㣬�޽��㷵��0
   vv = line(2,:)-line(1,:);
   t = dot(cir_n,o-line(1,:))/dot(cir_n,vv);
   if t>=0&&t<=1
       state = 1;
       po = line(1,:)+ vv*t;
   else
       state = 0;
       po=0;
   end
   
end