function [ ] = write_fetch_scene_to_csv(W, filename)
% takes in an example world
% writes .csv to be used in MoveIt

% create a matrix M
% first row: agent initial state
M = zeros(3 + W.N_obstacles, 6);
M(1, :) = W.start';
M(2, :) = W.goal';
M(3, :) = nan*ones(1, 6);

for i = 1:W.N_obstacles
   M(3 + i, :) =  [W.obstacles{i}.center', W.obstacles{i}.side_lengths];
end

if ~exist('filename', 'var')
    filename = ('scene_1.csv');
end
csvwrite(filename, M);
 
end

