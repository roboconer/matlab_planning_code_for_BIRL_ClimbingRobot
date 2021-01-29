function [ ret ] = inverse_wall( wall )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
ret = [ wall(4,:);
        wall(3,:);
        wall(2,:);
        wall(1,:);];
end

