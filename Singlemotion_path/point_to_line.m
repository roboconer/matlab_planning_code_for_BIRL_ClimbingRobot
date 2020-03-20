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

