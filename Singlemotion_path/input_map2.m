function [map_border, obstruct_border, obstruct_height] = input_map2()
   map_border = [];
  obstruct_border{1}=[0];
  obstruct_height{1}=[0];
obstruct_border{2}=[0];
obstruct_height{2}= [0];
obstruct_border{3}=[0];
  obstruct_height{3}= [0];
obstruct_border{4}=[0];
  obstruct_height{4}= [0];
  obstruct_border{5}=[0];
 obstruct_height{5}= [0];
  obstruct_border{1}=[0];
  obstruct_height{1}= [0];
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