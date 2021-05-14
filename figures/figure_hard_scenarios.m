% figure for arxiv version of the paper
% showing all 7 hard scenario scenes from the same view

clear; clc; close all;

world_file_header = 'scene';
world_file_folder = 'arm_planning/simulator_files/testing/saved_worlds/20200116_scenarios/';
world_file_location = sprintf('%s*%s*', world_file_folder, world_file_header);
world_file_list = dir(world_file_location);



blues = 1/256*[222,235,247
    158,202,225
    49,130,189];

lightpurple = 1/256*[117,107,177];
purple = 1/256*[84,39,143];

plot_CAD_flag = false;
verbosity = 0 ;
dimension = 3;
goal_type = 'configuration';
goal_radius = pi/30;


for idx = 1:length(world_file_list)
% for idx = 7
    
    figure(1); subplot(1, 7, idx); axis equal; hold on; grid off; box on; view(3);
%     figure(idx); subplot(1, 7, idx); clf; axis equal; hold on; grid off; box on; view(3);
    if idx == 4
        set(gca, 'XLim', [-1 1.5], 'YLim', [-1, 1.5], 'ZLim', [-0.05 1.5]);
    else
        set(gca, 'XLim', [-0.5 1.25], 'YLim', [-1, 1], 'ZLim', [-0.05 1.75]);
    end
    set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
    set(gca,'Projection','Perspective')
    
    
    world_filename = world_file_list(idx).name;
    [start, goal, obstacles] = load_saved_world([world_file_folder world_filename]);
    
    A = robot_arm_3D_fetch('verbose', verbosity, 'animation_set_axes_flag', 0, 'animation_set_view_flag', 0, 'plot_CAD_flag', plot_CAD_flag);
    A.link_plot_face_opacity = 0.5;
    A.link_plot_edge_opacity = 0.2;
    % A.link_plot_edge_color = blues(3, :);
    A.link_plot_face_color = blues(3, :);
    A.link_plot_face_opacity = 1;
    A.link_plot_edge_opacity = 0.2;
    A.link_plot_edge_color = lightpurple;
    A.link_plot_face_color = purple;
    
    agent_info = A.get_agent_info;
    
    W = fetch_base_world_static('create_random_obstacles_flag', false, 'include_base_obstacle', true, 'goal_radius', goal_radius, 'N_obstacles',length(obstacles),'dimension',dimension,'workspace_goal_check', 0,...
        'verbose',verbosity, 'start', start, 'goal', goal, 'obstacles', obstacles, 'goal_type', goal_type) ;
    
    for i = 1:length(W.obstacles)
        if W.obstacles{i}.is_base_obstacle
            W.obstacles{i}.plot_face_opacity = 0.2;
            %         W.obstacles{i}.plot_edge_opacity = 1;
            %         W.obstacles{i}.plot_face_opacity = 0.2;
            if ~(W.obstacles{i}.plot_face_color(1) == 0.9)
                W.obstacles{i}.plot_face_color = [0.7 0.7 0.7];
            else
                %             W.obstacles{i}.plot_face_color = [0 0 0];
            end
        else
            W.obstacles{i}.plot_face_opacity = 0.75;
            %         W.obstacles{i}.plot_edge_opacity = 1;
            W.obstacles{i}.plot_face_color = [1 0.7 0.7];
        end
    end
    
    A.state(A.joint_state_indices) = W.start ;
    W.setup(agent_info)
    plot(W); plot(A);


    
end

