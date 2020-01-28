function cabinet_cell = make_roahmlab_cabinet_obstacle(xy_location)
% cabinet_cell = make_roahmlab_cabinet_obstacle(xy_location)
%
% Creates a cell array of box_obstacle_zonotopes that mimic Cabinet 3 in
% ROAHM Lab.
%
% Author: Shreyas Kousik
% Created: 23 Jan 2019
% Updated: no
    
    if nargin < 1
        xy_location = [0.8;0] ;
    end

    % make sides
    left = box_obstacle_zonotope('center',[0;-0.82/2;1.94/2],...
        'side_lengths',[0.34 0.03 1.94]) ;
    right = box_obstacle_zonotope('center',[0;+0.82/2;1.94/2],...
        'side_lengths',[0.34 0.03 1.94]) ;
    back = box_obstacle_zonotope('center',[0.34/2;0;1.94/2],...
        'side_lengths',[0.04 0.84 1.94]) ;

    % make top and bottom
    top = box_obstacle_zonotope('center',[0;0;1.94],...
        'side_lengths',[0.34 0.84 0.04]) ;

    bottom = box_obstacle_zonotope('center',[0;0;0.025],...
        'side_lengths',[0.34 0.84 0.05]) ;

    cabinet_cell = {left, right, back, top, bottom} ;

    % make each shelf
    current_shelf_height = 0.295 ;
    shelf_height_delta = [0.295 0.28 0.38 0.29 0.25] ;
    
    for idx = 1:5
        shelf_idx = box_obstacle_zonotope('center',[0;0;current_shelf_height],...
            'side_lengths',[0.34 0.84 0.025]) ;

        cabinet_cell = [cabinet_cell, {shelf_idx}] ;
        current_shelf_height = current_shelf_height + shelf_height_delta(idx) ;
    end
    
    % shift everything to the right xy location
    for idx = 1:length(cabinet_cell)
        cabinet_cell{idx}.shift_center([xy_location(:) ; 0]) ;
    end
end