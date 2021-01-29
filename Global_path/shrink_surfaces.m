% function [shrink_shrink_surface,num_point_each_surface,num_surfaces,start_point,shrink_surfaces_end_point] = surfaces

function [shrink_surface, shrink_surfaces_start_point, shrink_surfaces_end_point] = shrink_surfaces
%全局壁面序列相应的壁面数据输入

    [surface,num_point_each_surface,num_surfaces,~,~] = surfaces;
    r = 0.1998;%class
    
    %对每个壁面做缩放
    for i=1:num_surfaces
        v1= (surface{i}(num_point_each_surface(i),:)-surface{i}(1,:))/norm(surface{i}(num_point_each_surface(i),:)-surface{i}(1,:));
        v2= (surface{i}(2,:)-surface{i}(1,:))/norm(surface{i}(2,:)-surface{i}(1,:));
        ang_cos = dot(v1,v2);
        ang_sin = sqrt(1-ang_cos^2);
        l = r/ang_sin;
        shrink_surface{i}(1,:) = surface{i}(1,:)+ v1*l +v2*l;

        v1=( surface{i}(1,:)-surface{i}(num_point_each_surface(i),:))/norm(surface{i}(1,:)-surface{i}(num_point_each_surface(i),:));
        v2= (surface{i}(num_point_each_surface(i)-1,:)-surface{i}(num_point_each_surface(i),:))/norm(surface{i}(num_point_each_surface(i)-1,:)-surface{i}(num_point_each_surface(i),:));
        ang_cos = dot(v1,v2);
        ang_sin = sqrt(1-ang_cos^2);
        l = r/ang_sin;
        shrink_surface{i}(num_point_each_surface(i),:) = surface{i}(num_point_each_surface(i),:)+ v1*l +v2*l;

        for j=2:num_point_each_surface(i)-1
            v1= (surface{i}(j+1,:)-surface{i}(j,:))/norm(surface{i}(j+1,:)-surface{i}(j,:));
            v2= (surface{i}(j-1,:)-surface{i}(j,:))/norm(surface{i}(j-1,:)-surface{i}(j,:));
            ang_cos = dot(v1,v2);
            ang_sin = sqrt(1-ang_cos^2);
            l = r/ang_sin;
            shrink_surface{i}(j,:) = surface{i}(j,:)+ v1*l +v2*l;
        end
    end
    
% 壁面缩放后，起始点和终止点
    shrink_surfaces_start_point = zeros(1,3);
    for i = 1 : num_point_each_surface(1)
        shrink_surfaces_start_point = shrink_surfaces_start_point + shrink_surface{1}(i,:);
    end
    shrink_surfaces_start_point = shrink_surfaces_start_point / num_point_each_surface(1);

%     shrink_surfaces_start_point = -0.05 *(shrink_surface{1}(1,:)-shrink_surface{1}(4,:)) + shrink_surfaces_start_point;
%     shrink_surfaces_start_point = -0.05 *(shrink_surface{1}(4,:)-shrink_surface{1}(3,:)) + shrink_surfaces_start_point;
     shrink_surfaces_start_point = 0.2 *(shrink_surface{1}(1,:)-shrink_surface{1}(4,:)) + shrink_surfaces_start_point;
     shrink_surfaces_start_point = 0.2 *(shrink_surface{1}(4,:)-shrink_surface{1}(3,:)) + shrink_surfaces_start_point;
    

    shrink_surfaces_end_point = zeros(1,3);
    for i = 1 : num_point_each_surface(num_surfaces)
        shrink_surfaces_end_point = shrink_surfaces_end_point + shrink_surface{num_surfaces}(i,:);
    end
    shrink_surfaces_end_point = shrink_surfaces_end_point / num_point_each_surface(num_surfaces);

%      shrink_surfaces_end_point = -0.15 * (shrink_surface{num_surfaces}(1,:)-shrink_surface{num_surfaces}(4,:)) + shrink_surfaces_end_point;
%      shrink_surfaces_end_point = -0.15 * (shrink_surface{num_surfaces}(4,:)-shrink_surface{num_surfaces}(3,:)) + shrink_surfaces_end_point;
    shrink_surfaces_end_point = 0.2 * (shrink_surface{num_surfaces}(1,:)-shrink_surface{num_surfaces}(4,:)) + shrink_surfaces_end_point;
    shrink_surfaces_end_point = 0.3 * (shrink_surface{num_surfaces}(4,:)-shrink_surface{num_surfaces}(3,:)) + shrink_surfaces_end_point;
end

