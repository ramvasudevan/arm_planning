function shelf_cell = make_shelf_obstacle(center,height,width,depth,N_shelves,min_shelf_height,max_shelf_height)

% default shelf thiccness
thickness = 0.01 ; % 1 cm


% make the left and right sides
left_side = {box_obstacle_zonotope('center',center + [0;-width/2;0],...
    'side_lengths',[depth thickness height],'creation_buffer',0.02)} ;

right_side = {box_obstacle_zonotope('center',center + [0;+width/2;0],...
    'side_lengths',[depth thickness height],'creation_buffer',0.02)} ;

if nargin < 5
    min_shelf_height = center(3) - height/2 ;
    max_shelf_height = center(3) + height/2 ;
end

% make the shelves
shelf_heights = linspace(min_shelf_height,...
    max_shelf_height,...
    N_shelves) ;
shelves = cell(1,N_shelves) ;

for idx = 1:N_shelves
    shelves{idx} = box_obstacle_zonotope('center',[center(1:2) ; shelf_heights(idx)],...
        'side_lengths',[depth width thickness],'creation_buffer',0.02) ;
end

shelf_cell = [left_side, right_side, shelves] ;
end