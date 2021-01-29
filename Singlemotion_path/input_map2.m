%%%%%%%%%%%这里的obstruct_border{i}可以存放多个障碍物数据，i同壁面序号，也就是第i个壁面的障碍物数据

function [map_border, obstruct_border, obstruct_height] = input_map2()
map_border = [];
width = 0.12;
length = 0.20;
location_1_x = 0.50;
location_1_y = 0.05;

width3 = 0.20;
length3 = 0.20;
location_3_x = 0.15;
location_3_y = 0.30;
% obstruct_border{1}=[0 0.4; 0.4 0.4; 0.4 0; 0 0];
% obstruct_height{1}=[0.5];
% obstruct_border{2}=[-0.25 0; -0.25 1; -1.2 1; -1.2 0;
%                     0.75 0; 0.76 0; 0.76 1; 0.75 1];
% obstruct_border{2}=[0.7 -0.25; 0.8 -0.25; 0.8 -0.3; 0.7 -0.3;];
% obstruct_border{2}=[ %0.3 -0.25; 0.9 -0.25; 0.9 -0.3; 0.3 -0.3;
%                     location_1_x location_1_y; location_1_x - length, location_1_y;
%                     location_1_x - length, location_1_y - width; 
%                     location_1_x location_1_y - width];
% obstruct_height{2}=[%0.15 
%     0.15];
% obstruct_border{3}=[location_3_x location_3_y; location_3_x - length3, location_3_y;
%                     location_3_x - length3, location_3_y - width3; 
%                     location_3_x location_3_y - width3];
% obstruct_height{3}=[0.2];

obstruct_border{1}=[0];
obstruct_height{1}=[0];
obstruct_border{2}=[-0 0.7; 0.0 0.7; 0.0 1.2; -0 1.2];
%obstruct_border{2}=[0.25 0; 0.25 1; 1.2 1; 1.2 0];
obstruct_height{2}=[0.1];
obstruct_border{3}=[0];
obstruct_height{3}=[0];
obstruct_border{4}=[0];
obstruct_height{4}=[0];
obstruct_border{5}=[0];
obstruct_height{5}=[0];

%  obstruct_border{2}=[0.5 0.2;0.6 0.2;0.6 0.4;0.5 0.4];
%   obstruct_height{2} = [0.5];
%  obstruct_border{2}=[-0.5 0.8;0 0.8;0 3;-0.5 3;0.8 1.1;0.93 1.1;0.93 2.2;0.8 2.2];
%  obstruct_height{2} = [0.2,0.15];
%  [ obstruct_border{2}(5:8,:) ] = ob_rotate( obstruct_border{2}(5:8,:),-30 );
%  
%  obstruct_border{3}=[-0.5,3;-0.25,3;-0.25,3.8;-0.5,3.8];
%   obstruct_height{3} = [0.3];
  

% obstruct_border{4}=[3.2 0.2;3.3 0.2;3.3 0.6;3.2 0.6];%2020/1/11 :将这个障碍物注释掉，换成0
% obstruct_height{4} = 0.25;%2020/1/11 :将这个障碍物注释掉，换成0
% [ obstruct_border{4}(1:4,:) ] = ob_rotate( obstruct_border{4}(1:4,:),-20 );%2020/1/11 :将这个障碍物注释掉，换成0
% obstruct_border{5}=[3.5 0.8;4.0 0.8;4.0 1.5;3.5 1.5];%2020/1/11 :将这个障碍物注释掉，换成0
% obstruct_height{5}= [0.4];%2020/1/11 :将这个障碍物注释掉，换成0
end