clear;

%% fetch wrrrrld stuff
A = robot_arm_3D_fetch() ;
W = fetch_base_world_static() ;
W.setup(A.get_agent_info)

%% ros stuff
fetch_ros_init

% set up subscribers
sub_fetch = rossubscriber('/mocap_fetch') ;
try
    msg_fetch = sub_fetch.receive(3) ;
    p_fetch = get_position_from_message(msg_fetch) ;
catch
    disp('fetch data not received!')
    p_fetch = [0;0;0.5] ;
end
sub_obs = rossubscriber('/mocap_obs') ;

%% obs data
% obs_box_size = [0.32,0.32,0.32] ;
% obs_box_size = [0.4;0.4;0.5] ; % for quadrotor
obs_box_size = [0.2; 0.2; 0.33];
o = box_obstacle_zonotope('side_lengths',obs_box_size,'center',zeros(3,1)) ;

% T_obs = [-0.15 ; -0.03 ; 0.8] ; %%% SET THIS TRANSFORM MANUALLY FOR EACH OBSTACLE
% T_obs = [0;-0.05;-0.13]; % box
% T_obs = [0;+0.01;0] ; % quadrotor
T_obs = [0;0;0];
fetch_origin_to_markers = [-0.25;0;0.95];
R = eul2rotm([pi/2 0 0],'xyz') ;

%% plotting setup
figure(1) ; clf ; axis equal ; grid on ; view(3) ; hold on ;
plot(W) ;
plot(A)
plot(o)

%% realtime mcloopyface
while true
        disp('getting obs mocap data')
        msg_obs = sub_obs.receive() ;
        msg_fetch = sub_fetch.receive() ;
        
        % get the difference between the two positions
%         d_obs = get_position_from_message(msg_obs) - p_fetch ;
        
        d_obs = R*get_position_from_message(msg_obs);
        mocap_origin_to_markers = R*get_position_from_message(msg_fetch);
        origin_shift = mocap_origin_to_markers - fetch_origin_to_markers;
%         mocap_origin_to_fetch_origin = mocap_origin_to_markers - fetch_markers_to_mocap_markers - fetch_origin_to_markers;
        
        % disp('transforming obs to wrrrld frame')
%         p_obs = T_obs + R_obs*d_obs ; % new center of obsicle
        p_obs = d_obs + T_obs - origin_shift; % new center of obsicle
        p_fetch = mocap_origin_to_markers - origin_shift;
        
        plot3(p_fetch(1), p_fetch(2), p_fetch(3), 'b.', 'MarkerSize', 40);
        
        % fix the obstacle center
        o.change_center(p_obs) ;

        %disp('updating plot')
        V = o.plot_data.body.Vertices ;
        mV = mean(V,1) ;
        o.plot_data.body.Vertices = V - mV + repmat(p_obs(:)',size(V,1),1) ;
%     catch
%     end
    pause(0.05)
end

%% helper function
function p = get_position_from_message(msg)
    p = [msg.Pose.Position.X ; msg.Pose.Position.Y ; msg.Pose.Position.Z]./1000 ;
end