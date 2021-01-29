% function [wall_surfaces,num_of_wall_vertex_for_each_surface,num_of_wall_surfaces,start_point,end_point] = surfaces
% %全局壁面序列相应的壁面数据输入
% 
% % border_vertexs1=[1.3107    1.2574    1.2286    1.2840;
% %                  -0.9047   -0.9053   0.8164   0.7961;
% %                  0.1994    1.2053    1.1960    0.2080]';
% % border_vertexs2=[0.7980    0.7942    0.0810    0.1100;
% %                  1.2456    1.2180    1.2336    1.2643;
% %                  0.3475    1.2607    1.2463    0.2528]';
% % border_vertexs3=[0.9099   -0.2128   -0.2515    0.9095;
% %                  0.1420    0.1324    0.8018    0.8859;
% %                  1.5297    2.1922    2.2043    1.5220]';
% % border_vertexs4=[-0.0022   -0.0143   0.8961    0.9096;
% %                  -1.4735   -1.4892   -1.4828   -1.4688;
% %                  0.2427    1.3660    1.3671    0.2540]';
% 
% % border_vertexs1=[
% %     -2.500    -2.5000    0.0000;
% %      1.000    -2.5000    0.0000;
% %      1.000     1.0000    0.0000;
% %     -2.500     1.0000    0.0000;
% % ];
% 
% border_vertexs1=[
%     -1.000     0.0000    0.0000;
%     -2.500     0.0000    0.0000;
%     -2.500    -2.0000    0.0000;
%     -1.000    -2.0000    0.0000;
% ];
% 
% % border_vertexs2=[
% %     2.2448    0.4394    1.2245;
% %     2.2887    0.4380    0.3549;
% %     2.3039    0.2746    0.1552;
% %     2.3552   -1.5162    0.2339;
% %     2.3125   -1.5727    1.1144;
% %     2.2959   -1.3898    1.3319;
% %     2.2431    0.4102    1.2748;
% % ];
% 
% % border_vertexs3=[
% % %     0.7756   -0.4754    2.2142;
% % %     1.9235   -0.4606    1.5097;
% % %     2.0555   -0.2920    1.4286;
% % %     2.0552    0.3674    1.4284;
% % %     1.8016    0.5263    1.5839;
% % %     0.7458    0.5171    2.2318;
% % %     0.7332   -0.3296    2.2402;
% %     0.7332   -0.3296    2.2402;
% %     0.7458    0.5171    2.2318;
% %     1.8016    0.5263    1.5839;
% %     2.0552    0.3674    1.4284;
% %     2.0555   -0.2920    1.4286;
% %     1.9235   -0.4606    1.5097;
% %     0.7756   -0.4754    2.2142;
% % ];
% 
% border_vertexs2 = [
% %     -0.791759, -0.910981,  0.331077;
% %     -0.797474, -1.693077,  0.334927;
% %     -0.733364, -1.683951,  1.294083;
% %     -0.732037, -0.900157,  1.286919;
% 
%     -0.786413, -1.742215,  1.239957;
%     -0.782197, -0.802192,  1.215549;
%     -0.830944, -0.803688,  0.411962;
%     -0.837651, -1.758119,  0.429097;
% ];
% 
% border_vertexs3 = [
% %     -1.342569, -1.869135,  0.460361;
% %     -2.096333, -1.889620,  0.438588;
% %     -2.130069, -1.903083,  1.796728;
% %     -1.372161, -1.875649,  1.803854;
% 
%     -2.192755, -1.914335, 1.731387;
%     -1.274996, -1.882555, 1.760319;
%     -1.247427, -1.878476, 0.533268
%     -2.199552, -1.907359, 0.535895;
% ];
%     
% border_vertexs4 = [  
% %     -2.700859, -1.819839,  0.311316;
% %     -2.697089, -0.085828,  0.338271;
% %     -2.664881, -0.096457,  1.340304;
% %     -2.680509, -1.830803,  1.303801;
% 
%     -2.719056, -1.895110,  0.354469;
%     -2.709321, -0.051001,  0.338329;
%     -2.683975, -0.040909,  1.301676;
%     -2.699243, -1.906624,  1.258313;
% ];
% 
% % border_vertexs3=[
% %     2.2941    0.4446    0.1835;
% %     2.3098    0.1199    0.0588;
% %     2.3550   -1.4585    0.1371;
% %     2.3127   -1.5681    1.1517;
% %     2.2938   -1.2824    1.3732;
% %     2.2464    0.3269    1.3224;
% %     2.2471    0.4395    1.2296;
% %     2.2924    0.4486    0.2198;
% %     2.0467   -0.4523    1.4486;
% %     0.7442   -0.5145    2.2271;
% %     0.6937    0.5300    2.2486;
% %     1.9872    0.5419    1.4760;
% %     2.0940    0.3962    1.4134;
% %     2.0908   -0.3507    1.4215;
% % ];
% 
% wall_surfaces{1}=border_vertexs2;
% wall_surfaces{2}=border_vertexs3;
% % wall_surfaces{3}=border_vertexs3;
% % wall_surfaces{4}=border_vertexs4;
% 
% % % wall_surfaces{4}=border_vertexs3; 
% % wall_surfaces{1}=border_vertexs4;
% % wall_surfaces{2}=border_vertexs1;
% % wall_surfaces{3}=border_vertexs2;
% % %wall_surfaces{4}=border_vertexs3; 
% 
% num_of_wall_surfaces=size(wall_surfaces,2);
% num_of_wall_vertex_for_each_surface=zeros(num_of_wall_surfaces,1);
% for i=1:num_of_wall_surfaces
%     num_of_wall_vertex_for_each_surface(i)=size(wall_surfaces{i},1);
% end
% 
% start_point = zeros(1,3);
% for i = 1 : num_of_wall_vertex_for_each_surface(1)
%     start_point = start_point + wall_surfaces{1}(i,:);
% end
% start_point = start_point / num_of_wall_vertex_for_each_surface(1);
% 
% start_point = -0.3*(wall_surfaces{1}(4,:)-wall_surfaces{1}(3,:)) + start_point;
% % start_point = [-2  -1  0];
% %start_point = [-1.5  -1  0];
% 
% end_point = zeros(1,3);
% for i = 1 : num_of_wall_vertex_for_each_surface(num_of_wall_surfaces)
%     end_point = end_point + wall_surfaces{num_of_wall_surfaces}(i,:);
% end
% %end_point = end_point / num_of_wall_vertex_for_each_surface(num_of_wall_surfaces);
% 
% end_point = 0.6*(wall_surfaces{num_of_wall_surfaces}(4,:)-wall_surfaces{num_of_wall_surfaces}(3,:)) + end_point;
% end_point = end_point/num_of_wall_vertex_for_each_surface(num_of_wall_surfaces);
% 
% end


function [wall_surfaces,num_of_wall_vertex_for_each_surface,num_of_wall_surfaces,start_point,end_point] = surfaces
%全局壁面序列相应的壁面数据输入

border_vertexs1=[
%     1.7500    -0.5000    0.0100;
%     2.7500    -0.5000    0.0100;
%     2.7500     1.7500    0.0100;
%     1.7500     1.7500    0.0100;
    
   0.149214 0 0.279136
   0.193918 0 1.285552
   2.547276 0 1.398885
   2.547276 0 0.279136
];

for i = 1:4
    border_vertexs1(i,:) = rotation_x_minus90(border_vertexs1(i,:));
end

border_vertexs2 = [

    0.193918 0.193682 1.285552
    0.149214 0.184601 0.279136
    0.139581 1.789117 0.264884
    0.173704 1.797810 1.271804
];

for i = 1:4
    border_vertexs2(i,:) = rotation_x_minus90(border_vertexs2(i,:));
end

border_vertexs3 = [
%     2.8260   -0.1218    1.9832
%     2.7707    1.8528    1.9957
%     2.7678    1.8648    1.2989
%     2.8284   -0.1149    1.2835

   2.547276 0.980600 1.398885
   0.541399 1.009244 1.427284
   0.558424 2.280701 1.416239
   2.561328 2.258063 1.391132
];

for i = 1:4
    border_vertexs3(i,:) = rotation_x_minus90(border_vertexs3(i,:));
end
    
border_vertexs4 = [  
%%%%%%%%%%%%内
-2.7191   -1.8951    0.3545
-2.7093   -0.0510    0.3383
-2.6840   -0.0409    1.3017
-2.6992   -1.9066    1.2583    

% % %%%%%%%%%%%%外
% -2.699243, -1.906624,  1.258313;
% -2.683975, -0.040909,  1.301676;
% -2.709321, -0.051001,  0.338329;
% -2.719056, -1.895110,  0.354469;

% %%外
% 1.00000, 0.51303021499, 2.60953893118;
% -1.00000, 0.51303021499, 2.60953893118;
% -1.000, 0 , 1.200;
% 1.000, 0, 1.200;

% %%内
% 1.000, 0, 1.200;
% -1.000, 0 , 1.200;
% -1.00000, 0.51303021499, 2.60953893118;
% 1.00000, 0.51303021499, 2.60953893118;
];

border_vertexs5 = [ 
%%%%%%%%%%%%内    
-2.709517, -1.786235, 1.456404
-2.698035, -0.069349, 1.466492
-1.24308,   0.342196, 1.992797
-1.206128, -1.629043, 1.942324

% %%%%%%%%%%%%外
% -1.2061   -1.6290    1.9423
% -1.2431    0.3422    1.9928
% -2.6980   -0.0693    1.4665
% -2.7095   -1.7862    1.4564

% %%%%外
% -1.000, 0, 0;
% 1.000, 0, 0;
% 1.000, 0, 1.200;
% -1.000, 0 , 1.200;

% %%%%内
% -1.000, 0 , 1.200;
% 1.000, 0, 1.200;
% 1.000, 0, 0;
% -1.000, 0, 0;

];

border_vertexs6 =[
    0         0         0;
    0    5.0000         0;
    2.0000    5.0000    2.3094;
    2.0000         0    2.3094;
    ];

border_vertexs7 =[
    2.1000         0    2.8000
    2.1000    5.0000    2.8000
    4.1000    5.0000   60.1074
    4.1000         0   60.1074
    ];

border_vertexs8 =[
     0     0     2
     0     2     2
     0     2     0
     0     0     0
    ];

border_vertexs9 =[
    1.0000         0    4.1445
    1.0000    2.0000    4.1445
         0    2.0000    2.0000
         0         0    2.0000
    ];

% wall_surfaces{1} = border_vertexs8;
% wall_surfaces{2} = border_vertexs9;
% 
% wall_surfaces{1} = inverse_wall(border_vertexs8);
% wall_surfaces{2} = inverse_wall(border_vertexs9);

% wall_surfaces{1}=border_vertexs1;
% wall_surfaces{2}=border_vertexs2;

% %内壁面
% wall_surfaces{1}=border_vertexs4;
% wall_surfaces{2}=border_vertexs5;

% % %外壁面
% wall_surfaces{1}=inverse_wall(border_vertexs4);
% wall_surfaces{2}=inverse_wall(border_vertexs5);

wall_surfaces{1}=border_vertexs1;
wall_surfaces{2}=border_vertexs2;
wall_surfaces{3}=border_vertexs3;
% wall_surfaces{4}=border_vertexs4;
% wall_surfaces{5}=border_vertexs5;

% wall_surfaces{5}=border_vertexs5;
% wall_surfaces{5}=border_vertexs1;

% wall_surfaces{1}=border_vertexs2;
% wall_surfaces{2}=border_vertexs3;
% wall_surfaces{3}=border_vertexs4;
% wall_surfaces{4}=border_vertexs4;

% wall_surfaces{1}=border_vertexs1;
% wall_surfaces{2}=border_vertexs2;
% wall_surfaces{3}=border_vertexs3;

num_of_wall_surfaces=size(wall_surfaces,2);
num_of_wall_vertex_for_each_surface=zeros(num_of_wall_surfaces,1);
for i=1:num_of_wall_surfaces
    num_of_wall_vertex_for_each_surface(i)=size(wall_surfaces{i},1);
end

start_point = zeros(1,3);
for i = 1 : num_of_wall_vertex_for_each_surface(1)
    start_point = start_point + wall_surfaces{1}(i,:);
end
start_point = start_point / num_of_wall_vertex_for_each_surface(1);

% start_point = -0.05 *(wall_surfaces{1}(1,:)-wall_surfaces{1}(4,:)) + start_point;
% start_point = -0.05 *(wall_surfaces{1}(4,:)-wall_surfaces{1}(3,:)) + start_point;
start_point = 0.2 *(wall_surfaces{1}(1,:)-wall_surfaces{1}(4,:)) + start_point;
start_point = 0.2 *(wall_surfaces{1}(4,:)-wall_surfaces{1}(3,:)) + start_point;


end_point = zeros(1,3);
for i = 1 : num_of_wall_vertex_for_each_surface(num_of_wall_surfaces)
    end_point = end_point + wall_surfaces{num_of_wall_surfaces}(i,:);
end
end_point = end_point / num_of_wall_vertex_for_each_surface(num_of_wall_surfaces);


% end_point = -0.15 * (wall_surfaces{num_of_wall_surfaces}(1,:)-wall_surfaces{num_of_wall_surfaces}(4,:)) + end_point;
% end_point = -0.15 * (wall_surfaces{num_of_wall_surfaces}(4,:)-wall_surfaces{num_of_wall_surfaces}(3,:)) + end_point;
 end_point = 0.2 * (wall_surfaces{num_of_wall_surfaces}(1,:)-wall_surfaces{num_of_wall_surfaces}(4,:)) + end_point;
 end_point = 0.3 * (wall_surfaces{num_of_wall_surfaces}(4,:)-wall_surfaces{num_of_wall_surfaces}(3,:)) + end_point;


end


