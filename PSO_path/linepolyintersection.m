function [inter_l,sta] =linepolyintersection(l1,poly,ln) 
%UNTITLED 求直线或线段与多边形交点,ln为1表示l1为直线，ln为2表示l1线段
num_poly=size(poly,1);
interpoint=zeros(1,2);
state=0;
inter_p=zeros(4,2);
 num_p =0;
 inter_l=zeros(2,2);
 for j=1:num_poly
        if j==num_poly
            l2=[poly(j,:);poly(1,:)];
        else
            l2=[poly(j,:);poly(j+1,:)];
        end
        if ln==1
    [interpoint,state] =lineintersection(l1,l2);
    if state == 1
        num_p=num_p+1;
    inter_p(num_p,:)=interpoint;
    end
        elseif ln==2
          [interpoint,state] =linesegintersection(l1,l2);  
           if state == 1
        num_p=num_p+1;
    inter_p(num_p,:)=interpoint;
           end
        end
 end
 if num_p>2
     sta =2;
    if norm(inter_p(1,:)-inter_p(2,:))>10^-5
        inter_l(1,:)=inter_p(1,:);
        inter_l(2,:)=inter_p(2,:);
    else
         inter_l(1,:)=inter_p(1,:);
        inter_l(2,:)=inter_p(3,:);   
    end
 elseif num_p==2
     if norm(inter_p(1,:)-inter_p(2,:))>10^-5
         sta=2;
        inter_l(1,:)=inter_p(1,:);
        inter_l(2,:)=inter_p(2,:);
     else
          sta=1;
        inter_l(1,:)=inter_p(1,:);
        inter_l(2,:)=inter_p(1,:);
     end
    elseif num_p==1
    sta=1;
    inter_l(1,:)=inter_p(1,:);
    inter_l(2,:)=inter_p(1,:);
 else
     sta=0;
 end
end