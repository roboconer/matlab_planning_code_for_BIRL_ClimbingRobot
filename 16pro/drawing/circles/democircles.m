%DEMOCIRCLES Demonstrate the CIRCLES function.
%   This script creates a random grid of circle positions, then plots the
%   circles as a single patch using the CIRCLES function. Various
%   combinations of color, transparency, and line properties are
%   demonstrated.
%
%   See also CIRCLES.

% Copyright 2016 Matthew R. Eicholtz
clear; clc; close all;

% Set the following parameters
alim = [200 200]; %[ax ay] -> axis limits, i.e. size of grid for circles
spacing = [10 10]; %[dx dy] -> circle spacing in x and y directions
minradius = 2; %minimum possible radius of each circle
maxradius = 5; %maximum possible radius of each circle

% Computing vectors of circle positions and radii
dx = spacing(1); dy = spacing(2);
[x,y] = meshgrid(dx:dx:alim(1)-1,dy:dy:alim(2)-1);
x = x(:);
y = y(:);

r = randi(maxradius-minradius+1,size(x))+minradius-1;

% Default settings
figure; set(gcf,'Name','Default settings');
axes; set(gca,'XTick',[],'YTick',[],'Box','on','XLim',[0 alim(1)],'YLim',[0 alim(2)]);
axis equal;
rect1 = circles(x,y,r);

% Shuffled colors for faces
figure; set(gcf,'Name','Shuffled face colors');
axes; set(gca,'XTick',[],'YTick',[],'Box','on','XLim',[0 alim(1)],'YLim',[0 alim(2)]);
axis equal;
rect2 = circles(x,y,r,'FaceColor','shuffle');

% Shuffled colors for edges
figure; set(gcf,'Name','Shuffled edge colors');
axes; set(gca,'XTick',[],'YTick',[],'Box','on','XLim',[0 alim(1)],'YLim',[0 alim(2)]);
axis equal;
rect3 = circles(x,y,r,'EdgeColor','shuffle');

% Blue circles with dotted orange edges
figure; set(gcf,'Name','Blue circles with dotted orange edges');
axes; set(gca,'XTick',[],'YTick',[],'Box','on','XLim',[0 alim(1)],'YLim',[0 alim(2)]);
axis equal;
rect4 = circles(x,y,r,'FaceColor','b','EdgeColor',[0.9294,0.6941,0.1255],...
    'LineStyle',':','LineWidth',2);

% Autumn circles with random transparency
figure; set(gcf,'Name','Autumn circles with random transparency');
axes; set(gca,'XTick',[],'YTick',[],'Box','on','XLim',[0 alim(1)],'YLim',[0 alim(2)]);
axis equal;
rect5 = circles(x,y,r,'FaceColor',autumn(length(x)),'EdgeColor','none',...
    'FaceAlpha',min(rand(length(x),1)+0.3,1));

