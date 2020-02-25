% script for generating animations from hardware demos
% Patrick Holmes
% 2020 01 28

clear; clc;

blues = 1/256*[222,235,247
158,202,225
49,130,189];

save_gif = false;

figure(1); clf; hold on; axis equal; grid on;
set(gca,'Projection','Perspective')
campos([16 -13 14])

filepath = 'arm_planning/fetch_command_files/fetch_hardware_demo_20200127/';
filename = 'fetch_hardware_table_1.mat';
gif_filename = ['gif_' filename(1:end-4) '.gif'];

load([filepath filename], 'P', 'W', 'A');
frame_rate_multiplier = 10;
frame_rate = frame_rate_multiplier*A.animation_time_discretization/A.animation_playback_rate;
start_gif = true;

A.link_plot_face_opacity = 1;
A.link_plot_edge_opacity = 0;
A.link_plot_edge_color = blues(3, :);
A.link_plot_face_color = blues(3, :);

% W = fetch_base_world_static('include_base_obstacle', 1);

plot(W);
patch_data = {};
slice_patch_data = {};
% colors = {'c', 'c', 'c'};
% slicecolors = {'m', 'm', 'm'};
colors = {blues(2, :), blues(2, :), blues(2, :)};
plot_RS_times = [100];
% plot_RS_times = [1,50,100];

camlight;
% light;

for i = 1:length(P.info.T)
    % plot obstacles
    for j = 1:length(P.info.obstacles{i}(5:end))
       plot(P.info.obstacles{i}{4+j}); 
    end
    
    % plot RSs if k_opt is not -12345
    if length(P.info.k_opt{i}) == 6
        R = robot_arm_FRS_rotatotope_fetch(P.info.q_0{i}, P.info.q_dot_0{i}, P.FRS_options);
        slice_patch_data = plot_slice_buffered(R, P.info.k_opt{i}, plot_RS_times, colors, slice_patch_data);
        %         patch_data = plot_buffered(R, plot_times, colors, slice_patch_data);
    end
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