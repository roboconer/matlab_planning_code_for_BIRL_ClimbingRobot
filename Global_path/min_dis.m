function f = min_dis(x)
% Pstart = [4.0656,6.91087,1.91813];
% Pend = [5.70673 6.3332 9.46937];
% Pstart = [2 2 0];
%  Pend = [2 4 4];
Pstart = zeros(1,3);
Pend = zeros(1,3);
[A,num_point,num_sur,Pstart,Pend] = surfaces;

   
   
% for i =1: num_point(1)
%     Pstart = Pstart+A{1}(i,:);
% end
% Pstart = Pstart/num_point(1);
% for i =1: num_point(num_sur)
%     Pend = Pend+ A{num_sur}(i,:);
% end
% Pend = Pend/num_point(num_sur);

numk = size(x,1);
s=0;

for i=1: numk-1
    s0 = norm(x(i+1,:)-x(i,:));
    s=s+s0;
end
f= norm(Pstart-x(1,:))+s+norm(Pend-x(numk,:));
end