%% make agent
A = robot_arm_3D_fetch() ;
AI = A.get_agent_info() ;

%% make world
start = [-pi/2.5;0;0;0;0;0] ;
goal = [pi/2.5;0;0;0;0;0] ;
W = fetch_base_world_static('start',start,'goal',goal) ;

%% make obstacle
% o = box_obstacle_zonotope('side_lengths',[0.1 0.1 2],'center',[0.7 0.0 1.0]') ; % 1
%o = box_obstacle_zonotope('side_lengths',[0.05 0.05 2],'center',[0.7 0.0 1.0]') ; % 2
% o = box_obstacle_zonotope('side_lengths',[0.3 0.3 2],'center',[0.7 0.0 1.0]') ; % 3
% o = box_obstacle_zonotope('side_lengths',[0.3 0.3 0.3],'center',[0.7 0.0 0.8]') ; % 4

o1 = box_obstacle_zonotope('side_lengths',[0.3 0.3 0.3],'center',[0.7 0.0 0.8]') ; % 5
o2 = box_obstacle_zonotope('side_lengths',[0.3 0.3 0.3],'center',[0.7 0.0 1.05]') ; % 5
t = make_roahmlab_table_obstacle([0.8;0]) ; % 5

W.add_obstacle([{o1, o2},t])

%% set up arm
W.setup(AI)
A.reset(W.start) ;

%% save csv
% filename = 'fetch_chomp_crash_test_5.csv' ;
% write_fetch_scene_to_csv(W, filename);

%% plotting
figure(1) ; clf ; axis equal ; hold on ; view(3) ; grid on ;
plot(W)
plot(A)

%% replay the thing from chomp
A = reset_agent_with_chomp_traj(A,'chomp_trajectory_crash_test_5.txt') ;
animate(A)
