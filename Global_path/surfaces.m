function [wall_surfaces,num_of_wall_vertex_for_each_surface,num_of_wall_surfaces,start_point,end_point] = surfaces
%全局壁面序列相应的壁面数据输入

border_vertexs1=[1.3107    1.2574    1.2286    1.2840;
   -0.9047   -0.9053    0.8164    0.7961  ;
    0.1994    1.2053    1.1960    0.2080  ]';
border_vertexs2=[0.7980    0.7942    0.0810    0.1100  ;
    1.2456    1.2180    1.2336    1.2643  ;
    0.3475    1.2607    1.2463    0.2528  ]';
border_vertexs3=[ 0.9099   -0.2128   -0.2515    0.9095  ;
    0.1420    0.1324    0.8018    0.8859  ;
    1.5297    2.1922    2.2043    1.5220  ]';
border_vertexs4=[-0.0022   -0.0143    0.8961    0.9096;
   -1.4735   -1.4892   -1.4828   -1.4688;
    0.2427    1.3660    1.3671    0.2540]';

wall_surfaces{1}=border_vertexs4;
wall_surfaces{2}=border_vertexs1;
wall_surfaces{3}=border_vertexs2;
%wall_surfaces{4}=border_vertexs3; 

num_of_wall_surfaces=size(wall_surfaces,2);
num_of_wall_vertex_for_each_surface=zeros(num_of_wall_surfaces,1);
for i=1:num_of_wall_surfaces
    num_of_wall_vertex_for_each_surface(i)=size(wall_surfaces{i},1);
end

start_point = zeros(1,3);
for i =1: num_of_wall_vertex_for_each_surface(1)
    start_point = start_point+wall_surfaces{1}(i,:);
end
start_point = start_point/num_of_wall_vertex_for_each_surface(1);

% start_point = 0.3*(wall_surfaces{1}(3,:)-wall_surfaces{1}(4,:)) +start_point;

end_point = zeros(1,3);
for i =1: num_of_wall_vertex_for_each_surface(num_of_wall_surfaces)
    end_point = end_point+ wall_surfaces{num_of_wall_surfaces}(i,:);
end
%end_point = end_point/num_of_wall_vertex_for_each_surface(num_of_wall_surfaces);

end_point = 0.5*(wall_surfaces{3}(4,:)-wall_surfaces{3}(3,:)) +end_point;
end_point = end_point/num_of_wall_vertex_for_each_surface(num_of_wall_surfaces);

end

