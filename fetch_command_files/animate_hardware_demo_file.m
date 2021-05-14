% script for generating animations from hardware demos
% Patrick Holmes
% 2020 01 28

clear; clc;

blues = 1/256*[222,235,247
158,202,225
49,130,189];

save_gif = true;
moving_obstacle_flag = true;


figure(1); clf; hold on; axis equal; grid off; box on;
% set(gcf, 'Color', 'k'); set(gca, 'Color', 'k');
% set(gcf, 'Color', 'm');
set(gca,'Projection','Perspective')
set(gca, 'XLim', [-0.5 1.5], 'ZLim', [-0.05 1.5]);
set(gca, 'XTick', [], 'YTick', [], 'ZTick', []);
% set(gca, 'YLim', [-0.75 0.75]);
% campos([16 -13 14])
% campos([  -10.3529    5.8647   15.9855]);
% campos([   -0.3872  -23.6489    0.8049]);
% campos([  -13.6903   13.1920    8.3689]);
% campos([18.7257   -0.0885    7.9968]);
% campos([   17.8319   -2.1317    9.6762]);
% campos([  -15.1827    6.7648   10.3870]);
campos([   17.1326   -5.3992    9.6283]);
% campos([   -9.2848   -0.0135   17.7344]);


filepath = 'arm_planning/fetch_command_files/fetch_hardware_demo_20200128/';
filename = 'fetch_hardware_vase_6.mat';
gif_filename = ['gif_' filename(1:end-4) '_sudden' '.gif'];

load([filepath filename], 'P', 'W', 'A');
frame_rate_multiplier = 4;
frame_rate = frame_rate_multiplier*A.animation_time_discretization/A.animation_playback_rate;
start_gif = true;

A.link_plot_face_opacity = 1;
A.link_plot_edge_opacity = 0.2;
% A.link_plot_edge_color = blues(3, :);
A.link_plot_face_color = blues(3, :);

% W.obstacles{4}.center(3) = W.obstacles{4}.center(3) + 0.0175;
% W.obstacles{4}.side_lengths(3) = W.obstacles{4}.side_lengths(3) - 0.0175;
% W.obstacles{4}.side_lengths(3) = W.obstacles{4}.side_lengths(3) - 0.02;
% W.obstacles{4}.shift_center([0;0;0.04]);

% W = fetch_base_world_static('include_base_obstacle', 1);
for i = 1:length(W.obstacles)
    if W.obstacles{i}.is_base_obstacle
        W.obstacles{i}.plot_face_opacity = 1;
        W.obstacles{i}.plot_edge_opacity = 1;
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

% update goal?
% W.goal = A.state(A.joint_state_indices, end);
% W.goal_plot_patch_data = A.get_collision_check_volume(W.goal) ;

plot(W);
patch_data = {};
slice_patch_data = {};
% colors = {'c', 'c', 'c'};
% slicecolors = {'m', 'm', 'm'};
colors = {blues(2, :), blues(2, :), blues(2, :)};
% plot_RS_times = [100];
plot_RS_times = [1,50,100];

% camlight;
% light;

% set up waypoint plotting
waypointcolor = [0.3 0.3 0.3];
waypointedgecolor = [0.2 0.2 0.2];
waypointedgealpha = 0.5;
% waypointlinestyle = '-';
% waypointlinewidth = 5;
% p_way = [];
A_way = robot_arm_3D_fetch('verbose', 0, 'animation_set_axes_flag', 0,...
    'animation_set_view_flag', 0,...
    'link_plot_face_color', waypointcolor, 'link_plot_edge_color', waypointedgecolor, ...
    'link_plot_edge_opacity', waypointedgealpha);

pause();
p_obs = [];
for i = 1:length(P.info.T)
    % plot obstacles
    if i >= 9
    if moving_obstacle_flag
        for j = 5:length(P.info.obstacles{i})
            if isempty(p_obs)
               p_obs = plot(P.info.obstacles{i}{j}); 
            else
                create_plot_patch_data(P.info.obstacles{i}{j});
                p_obs.Faces = P.info.obstacles{i}{j}.plot_patch_data.faces;
                p_obs.Vertices = P.info.obstacles{i}{j}.plot_patch_data.vertices;
%                disp('hi');
            end
        end
    end
    end
    
    % plot RSs if k_opt is not -12345
    if length(P.info.k_opt{i}) == 6
        R = robot_arm_FRS_rotatotope_fetch(P.info.q_0{i}, P.info.q_dot_0{i}, P.FRS_options);
        slice_patch_data = plot_slice_buffered(R, P.info.k_opt{i}, plot_RS_times, colors, slice_patch_data);
        %         patch_data = plot_buffered(R, plot_times, colors, slice_patch_data);
    end
    
    % plot waypoint
%     X_way = A.get_joint_locations_from_configuration(P.info.waypoint{i});
%     if isempty(p_way)
%         p_way = plot3(X_way(1, :), X_way(2, :), X_way(3, :), 'Color', waypointcolor, 'LineStyle', waypointlinestyle,...
%             'LineWidth', waypointlinewidth);
%     else
%         p_way.XData = X_way(1, :);
%         p_way.YData = X_way(2, :);
%         p_way.ZData = X_way(3, :);
%     end
    A_way.reset(P.info.waypoint{i});
    plot(A_way);
    
    
    % plot arm moving
    t_plan_idx = 50;
    for j = 1:frame_rate_multiplier:t_plan_idx
        A.plot_links(P.info.Z{i}(A.joint_state_indices, j));
        % create gif
        if save_gif
            % get current figure
            fh = get(groot,'CurrentFigure') ;
            frame = getframe(fh) ;
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            
            if start_gif
                imwrite(imind,cm,gif_filename,'gif', 'Loopcount',inf,...
                    'DelayTime',frame_rate) ;
                start_gif = false ;
            else
                imwrite(imind,cm,gif_filename,'gif','WriteMode','append',...
                    'DelayTime',frame_rate) ;
            end
        else
            pause(frame_rate)
        end
    end
    
end