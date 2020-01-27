function [table_cell] = make_roahmlab_table_obstacle(xy_location)
% cabinet_cell = make_roahmlab_cabinet_obstacle(xy_location)
%
% Creates a cell array of box_obstacle_zonotopes that mimic Cabinet 3 in
% ROAHM Lab.
%
% Author: Shreyas Kousik
% Adapted: Patrick Holmes
% Created: 26 Jan 2020
% Updated: no
    
    if nargin < 1
        xy_location = [0;0] ;
    end
    
    table_top = box_obstacle_zonotope('center', [0; 0; 0.7239],...
        'side_lengths', [0.7620 2.0320 0.0254]);
    
    table_left_leg = box_obstacle_zonotope('center', [0; -0.7683; 0.7239/2],...
        'side_lengths', [0.0762 0.0508 0.7239]);
    
    table_right_leg = box_obstacle_zonotope('center', [0; +0.7683; 0.7239/2],...
        'side_lengths', [0.0762 0.0508 0.7239]);
    
    table_cell = {table_top, table_left_leg, table_right_leg} ;

    % shift everything to the right xy location
    for idx = 1:length(table_cell)
        table_cell{idx}.shift_center([xy_location(:) ; 0]) ;
    end
end