%% Basic RRT functionality tests
% Daphna Raz 2019

%% 2D tests
% dimension implicity set by size of goal
goal = [0 pi/2]';
RRT_object = robot_arm_RRT_HLP('goal', goal);

%% test generate_random_config(HLP, bounds)
bounds = [0 -2*pi;
          pi 2*pi;];

q_rand = RRT_object.generate_random_config(bounds);

for i = 1:RRT_object.dimension
    assert(q_rand(i) > bounds(1,i) & q_rand(i) < bounds(2,i))
end
    

%% test find_nearest_node(HLP, q_rand)
node1 = [0 0]';
node2 = [0 pi/2]';
node3 = [2*pi 2*pi]';

nodes = [node1 node2 node3];

RRT_object.nodes = nodes;
RRT_object.node_parent_idx = [0 1 2];
q_near_idx = RRT_object.find_nearest_node(goal);
assert(q_near_idx == 2);

q_near = RRT_object.nodes(1:RRT_object.dimension, q_near_idx);

%% test generate_new_config(HLP, q_rand, q_near_idx)

q_new = RRT_object.generate_new_config(q_rand,q_near_idx);

% verify that q_rand, q_near, and q_new are collinear
A = [q_rand-q_near q_near-q_new];

assert(rank(A) < 2);




