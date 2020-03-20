function [ sta ] = circlepoly_inter( o,r,cir_n,poly )
%求空间圆与多边形是否相交，相交返回1，否则为0,输入为圆心，半径，圆所在平面法向和多边形
num_p = size(poly,1);
twoside = 0;
onp = 0;
onpoi = zeros(2,3);
num_inter = 1;
interline = zeros(2,3);
inp = zeros(4,3);
for i=1:num_p
    if i==num_p
       if dot(cir_n,poly(i,:)-o)*dot(cir_n,poly(1,:)-o)<1e-5
        twoside = twoside +1;
        break;
       end
    else
       if dot(cir_n,poly(i,:)-o)*dot(cir_n,poly(i+1,:)-o)<1e-5
        twoside = twoside +1;
        break;
        end 
    end
    
    if dot(poly(i,:)-o,cir_n)<1e-4
        onp = onp +1;
        onpoi(onp,:) = poly(i,:);
    end
end

if twoside ==1
    for i = 1:num_p
    if i==num_p
        line=[poly(i,:);poly(1,:)];
    else
              line=[poly(i,:);poly(i+1,:)];
    end
       
        if abs(dot(line(2,:)-line(1,:),cir_n))<1e-5
            continue;
        end
        [state,po] = linesegtosur(line,o,cir_n);
        if state ==1
            inp(num_inter,:) = po;
            num_inter = num_inter +1;
        end
    end
   
    if num_inter>3
        if norm(inp(1,:)-inp(2,:))<1e-5
            interline=[inp(1,:);inp(3,:)];
        end
    else 
       interline=[inp(1,:);inp(2,:)]; 
    end
elseif twoside == 0 && onp ==2
    interline=[onpoi(1,:);onpoi(2,:)];
elseif twoside ==0 && onp ==1
    interline=[onpoi(1,:);onpoi(1,:)];
else
    sta = 0;
    return;
end
[dis,lp] = point_to_line(o,interline);
    if dis <=r
        sta = 1;
    else
        sta = 0;
    end
end

function [state,po] = linesegtosur(line,o,cir_n)%线段与面求交点，无交点返回0
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

function [dis,lp] = point_to_line(poi,line)   %求点到线段的最小距离，返回距离及对应的点
   if norm(line(1,:) - line(2,:))<1e-6
       dis = norm(line(1,:) - poi);
       lp = line(1,:);
       return;
   end
   vk = line(2,:) - line(1,:);
   v1 = poi - line(1,:);
   v2 = poi - line(2,:);
   if dot(vk,v1) < 1e-6
       lp = line(1,:);
       dis = norm(v1);
       return;
   end
   if dot(vk,v2) > 1e-6
       lp = line(2,:);
       dis = norm(v2);
       return;
   end
       dis = norm(cross(vk,v1)/norm(vk));
       lp = line(1,:) + vk*(dot(vk,v1)/dot(vk,vk));  
end
