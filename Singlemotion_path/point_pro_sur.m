function [prop] = point_pro_sur(point,poly)      %点在平面上的投影
vec = cross(poly(2,:)-poly(1,:),poly(3,:)-poly(2,:));
vec = vec/norm(vec);
mean_p = zeros(1,3);
num_p  = size(poly,1);
mean_p = mean_p/num_p;

for i=1:num_p
mean_p = mean_p + poly(i,:);
end
prop = point-vec*dot(point-poly(1,:),vec);
end