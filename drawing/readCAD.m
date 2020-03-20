%% ==========================================================
% 函数功能：读入ASC II编码的STL格式CAD零件
% based on codes by Eric Trautmann etrautmann@gmail.com
% 输入: filename - 文件名字，字符串
%
% 输出： faces - 面
%       vertices - 顶点

function [faces,vertices]=readCAD(filename)

%open file
fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.
if fid == -1
    error('File could not be opened, check name or path.')
end

%=====================================================

fid=fopen(filename, 'r'); %Open the file, assumes STL ASCII format.

fmt = '%*s %*s %*f32 %*f32 %*f32 \r\n %*s %*s \r\n %*s %f32 %f32 %f32 \r\n %*s %f32 %f32 %f32 \r\n %*s %f32 %f32 %f32 \r\n %*s \r\n %*s \r\n';
C=textscan(fid, fmt, 'HeaderLines', 1);
fclose(fid);

% extract normal vectors and vertices
% tnorm = cell2mat(C(1:3));
% tnorm = tnorm(1:end-1,:); %strip off junk from last line

v1 = cell2mat(C(1:3));
v2 = cell2mat(C(4:6));
v3 = cell2mat(C(7:9));

if (isnan(C{1}(end)))  % ~isnan ? -- HFZHU 2016/8/5
    v1 = v1(1:end-1,:); % strip off junk from last line
    v2 = v2(1:end-1,:); % strip off junk from last line
    v3 = v3(1:end-1,:); % strip off junk from last line
end

v_temp = [v1 v2 v3]';
vnum = numel(v_temp)/3;
vertices = zeros(3,vnum);

vertices(:) = v_temp(:);
vertices = vertices';

% fnum = length(vertices)/3;      % Number of faces, vnum is number of vertices.  STL is triangles.
flist = 1:vnum;     %Face list of vertices, all in order.
faces = reshape(flist, 3,length(vertices)/3); %Make a "3 by fnum" matrix of face list data.
%
%   Return the faces and vertexs.
%
faces = faces';  % Orients the array for direct use in patch.

end