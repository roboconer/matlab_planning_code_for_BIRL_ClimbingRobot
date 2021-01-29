function [ d ] = point_line_distance( Q1, Q2, P )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
d = norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1);
end

