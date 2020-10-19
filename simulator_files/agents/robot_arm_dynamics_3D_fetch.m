classdef robot_arm_dynamics_3D_fetch < robot_arm_agent
    %% properties
    properties
        % hacking stuff
        k_opt = [];
        
        Z = [];
        
        % actual initial conditions
        q_act_0 = [];
        q_dot_act_0 = [];
        
        % desired initial conditions
        q_des_0 = [];
        q_dot_des_0 = [];
        
        % whether or not to plot the CAD version of the arm
        use_CAD_flag = false ;
        link_CAD_data = [] ;
        
        % joint axes
        joint_axes_CAD = [0 0 1 0 1 0 1 1 ;
            0 1 0 1 0 1 0 0 ;
            1 0 0 0 0 0 0 0 ] ;
        
        % joint locations
        joint_locations_CAD = [-0.03 0.12 0.21 0.13 0.20 0.12 0.14 0.16 ;
            0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 ;
            0.75 0.06 0.00 0.00 0.00 0.00 0.00 0.00 ;
            0 0 0 0 0 0 0 0 ;
            0 0 0 0 0 0 0 0 ;
            0 0 0 0 0 0 0 0 ] ;
    end
    
    %% methods
    methods
        function A = robot_arm_dynamics_3D_fetch(varargin)
            dimension = 3 ;
                      
            n_links_and_joints = 6 ;
            
            n_states = 2*n_links_and_joints ;
            
            link_shapes = {'cylinder', 'cylinder','cylinder','cylinder','cylinder', 'cylinder'} ;
            
%             link_sizes = [0.05, 0.300, 0.05, 0.200 ;  % size in x
%                           0.05, 0.025, 0.05, 0.025 ;  % size in y
%                           0.05, 0.025, 0.05, 0.025] ; % size in z

            buffer_dist = 0.150;
%             buffer_dist = 0.1460*sqrt(2);


            % for rectangular links:
%             link_sizes = [0.1206, 0.4635, 0.001, 0.4254, 0.001, 0.3810 ;  % size in x
%                           0.1460, 0.1460, 0.001, 0.150, 0.001, 0.1460 ;  % size in y
%                           0.0825, 0.1460, 0.001, 0.150, 0.001, 0.1460] ; % size in z
            
            % for cylindrical links:
            link_sizes = [0.1206, 0.4635, 0.001, 0.4254, 0.001, 0.3810 ;  % size in x
                          0.1460, 0.1460, 0.001, 0.150, 0.001, 0.1460 ;  % size in y
                          0.1460, 0.1460, 0.001, 0.150, 0.001, 0.1460] ; % size in z

            joint_state_indices = 1:2:n_states ;
            
            joint_speed_indices = 2:2:n_states ;
            
            joint_types = {'revolute','revolute','revolute','revolute', 'revolute', 'revolute'} ;
            
            joint_axes = [0 0 1 0 1 0;
                          0 1 0 1 0 1;
                          1 0 0 0 0 0] ; % axes are in preceding link's frame
                      
            joint_locations = [-0.03265, +0.1206/2, +0.3556/2, +0.00, +0.3302/2, +0.00 ; % predecessor x
                               +0.00, +0.00, +0.00, +0.00, +0.00, +0.00 ; % predecessor y
                               +0.72601, +0.0825/2, +0.00, +0.00, +0.00, +0.00 ; % predecessor z
                               -0.1206/2, -0.3556/2, +0.00, -0.3302/2, +0.00, -0.1460 ; % successor x
                               +0.00, +0.00, +0.00, +0.00, +0.00, +0.00 ; % successor y
                               -0.0825/2, +0.00, +0.00, +0.00, +0.00, +0.00 ];% successor z
                           
            joint_state_limits = [-1.6056, -1.221, -Inf, -2.251, -Inf, -2.16;
                                  1.6056, 1.518, Inf, 2.251, Inf, 2.16];
                        
            joint_speed_limits = (Inf).*repmat([-1;1],1,n_links_and_joints) ;
            
            joint_input_limits = 3.*joint_speed_limits;
            
            kinematic_chain = [0 1 2 3 4 5;
                               1 2 3 4 5 6] ;
                           
            gravity_direction = [0;0;-1] ;
            
            set_view_when_animating = true ;
            
            animation_view = 3 ;
            
            A@robot_arm_agent('dimension',dimension,'n_links_and_joints',n_links_and_joints,...
                'n_links_and_joints',n_links_and_joints,'n_inputs',n_links_and_joints,...
                'n_states',n_states,...
                'link_shapes',link_shapes,'link_sizes',link_sizes,...
                'joint_state_indices',joint_state_indices,...
                'joint_speed_indices',joint_speed_indices,...
                'joint_types',joint_types,'joint_axes',joint_axes,...
                'joint_locations',joint_locations,...
                'joint_state_limits',joint_state_limits,...
                'joint_speed_limits',joint_speed_limits,...
                'joint_input_limits',joint_input_limits,...
                'kinematic_chain',kinematic_chain,...
                'gravity_direction',gravity_direction,...
                'animation_set_view_flag',set_view_when_animating,...
                'animation_view',animation_view,...
                'buffer_dist', buffer_dist,...
                'animation_time_discretization', 0.01,...
                varargin{:}) ;
            
            if A.use_CAD_flag
                A.load_CAD_arm_patch_data()
                A.link_plot_edge_color = [0 0 1] ;
                A.link_plot_edge_opacity = 0 ;
                A.link_plot_face_color = [0.8 0.8 1] ;
                A.link_plot_face_opacity = 1 ;
            end
        end
        
%         %% dynamics
%         function zd = dynamics(A,t,z,T,U,Z) 
%             % SUPER HACKY WAY TO COMPUTE THE DYNAMICS
%             A.Z = Z;
%             % get joint speeds
%             z = z(:);
%             q = z(1:2:end);
%             dq = z(2:2:end); 
%             
%             dq = bound_array_elementwise(dq,...
%                 A.joint_speed_limits(1,:)',A.joint_speed_limits(2,:)') ;
%             
%             % get reference joint speeds
%             [qdes,dqdes,ddqdes] = A.desired_dynamics(t);
% 
%             % preallocate dynamics
%             zd = zeros(A.n_states,1);
%             
%             % controller gains
%             Kp = 32^2*eye(6);
%             Kd = 64*eye(6);
%             
%             Kp = 1000*eye(6);
%             Kd = 200*eye(6);
%             
%             % generate dynamics matrices
%             [H, C] = fetch_dynamics(q ,dq);
%             
%             % calculate control inputs
%             tau = H*(ddqdes - Kp*(q-qdes) - Kd*(dq-dqdes)) + C;
%             
%             % bound control inputs
%             tau = bound_array_elementwise(tau,...
%                                           A.joint_input_limits(1,:)',...
%                                           A.joint_input_limits(2,:)') ;
%             
%             % calculate joint accelrations
%             ddq = H\(tau-C);
%             
%             % return dynamics
%             zd(A.joint_state_indices) = dq(:) ;
%             zd(A.joint_speed_indices) = ddq(:) ;
%         end
%         
%         function[q, dq, ddq ]= desired_dynamics(A, t)
%             t_to_plan = 0.5;
%             t_to_stop = 0.5;
%             
%             if t<= t_to_plan
%                 q = A.q_des_0 + A.q_dot_des_0 * t + (1/2)*A.k_opt * t^2;
%                 dq = A.q_dot_des_0 + A.k_opt * t;
%                 ddq = A.k_opt;
%             else
%                 t = t - t_to_plan;
%                 q_peak = A.q_des_0 + A.q_dot_des_0 * t_to_plan + (1/2)*A.k_opt * t_to_plan^2;
%                 q_dot_peak = A.q_dot_des_0 + A.k_opt * t_to_plan;
%                 
%                 q = q_peak + q_dot_peak*t + (1/2)*((0 - q_dot_peak)/t_to_stop)*t^2;
%                 dq = q_dot_peak + ((0 - q_dot_peak)/t_to_stop)*t;
%                 ddq = (0 - q_dot_peak)/t_to_stop;
%             end
%         end
%   
%         %% integrator
%         function [t_out,z_out] = integrator(A,arm_dyn,t_span,z0)
%             % [tout,zout] = A.integrator(arm_dynamics,tspan,z0)
%             %
%             % RK4 integration with joint limits and speed limits enforced.
%             
%             % create time vector
%             dt = A.integrator_time_discretization ;
%             t_out = t_span(1):dt:t_span(end) ;
%             if t_out(end) < t_span(end)
%                 t_out = [t_out, t_span(end)] ;
%             end
%             
%             % preallocate trajectory output
%             N_t = size(t_out,2) ;
%             z_out = [z0(:), nan(A.n_states,N_t-1)] ;
%             
%             % run integration loop
%             for tidx = 2:N_t
%                 % get previous state
%                 z_cur = z_out(:,tidx-1) ;
%                 t_cur = t_out(tidx-1) ;
%                 
%                 % compute RK4 terms
%                 k1 = arm_dyn(t_cur, z_cur) ;
%                 k2 = arm_dyn(t_cur + dt/2, z_cur + dt*k1/2) ;
%                 k3 = arm_dyn(t_cur + dt/2, z_cur + dt*k2/2) ;
%                 k4 = arm_dyn(t_cur + dt, z_cur + dt*k3) ;
%                 
%                 % compute summed term
%                 dzdt = (1/6)*(k1 + 2*k2 + 2*k3 + k4) ;
%                 
%                 % compute next state
%                 z_new = z_cur + dt*dzdt ;
%                 
%                 % apply state limits
%                 joint_values = z_new(A.joint_state_indices)' ;
%                 joint_values = max([joint_values ; A.joint_state_limits(1,:)],[],1) ;
%                 joint_values = min([joint_values ; A.joint_state_limits(2,:)],[],1) ;
%                 z_new(A.joint_state_indices) = joint_values ;
%                 
%                 % apply speed limits
%                 joint_speeds = z_new(A.joint_speed_indices)' ;
%                 joint_speeds = max([joint_speeds; A.joint_speed_limits(1,:)],[],1) ;
%                 joint_speeds = min([joint_speeds; A.joint_speed_limits(2,:)],[],1) ;
%                 z_new(A.joint_speed_indices) = joint_speeds ;
%                 
%                 % save new state
%                 z_out(:,tidx) = z_new(:) ;
%             end
%         end
       
%% plotting setup
        function [faces,vertices] = create_baselink_plot_patch_data(A)
            % create baselink cone for plotting
            [faces,vertices] = make_cuboid_for_patch(0.025, 0.025, 0.025, [0;0;0]) ;
        end
        
        function load_CAD_arm_patch_data(A)
            A.vdisp('Loading CAD files for arm plotting!',1)
            
            shoulder_pan = stlread('shoulder_pan_link_collision.STL') ;
            shoulder_lift = stlread('shoulder_lift_link_collision.STL') ;
            upper_arm = stlread('upperarm_roll_link_collision.STL') ;
            elbow = stlread('elbow_flex_link_collision.STL') ;
            forearm = stlread('forearm_roll_link_collision.STL') ;
            wrist_flex = stlread('wrist_flex_link_collision.STL') ;
            wrist_roll = stlread('wrist_roll_link_collision.STL') ;
            gripper = stlread('gripper_link.STL') ;
            
            temp_link_CAD_data = {shoulder_pan, shoulder_lift,...
                upper_arm, elbow, forearm, wrist_flex, wrist_roll, gripper} ;
            
            % check to make sure the CAD data are patches, not
            % triangulations
            triangulated_flag = false ;
            for idx = 1:8
                current_data = temp_link_CAD_data{idx} ;
                if isa(current_data,'triangulation')
                    triangulated_flag = true ;
                    new_data.faces = current_data.ConnectivityList ;
                    new_data.vertices = current_data.Points ;
                    temp_link_CAD_data{idx} = new_data ;
                end
            end
            
            if triangulated_flag
                A.vdisp('STL read returned a triangulated data format, but we fixed it :)',7)
            end
            
            A.link_CAD_data = temp_link_CAD_data ;
        end
        
        %% plotting
        function plot_links(A,time_or_config)
            if A.use_CAD_flag
                % get the rotations and translations at the current time
                if length(time_or_config) == 1
                    q = match_trajectories(time_or_config,A.time,A.state(A.joint_state_indices,:)) ;
                else
                    q = time_or_config ;
                end
                
                q = [q ; zeros(2,1)] ;
                
                % get transformations for the plot links
                [R,T] = get_link_rotations_and_translations_from_arm_data(q,...
                    A.joint_axes_CAD,A.joint_locations_CAD) ;
                
                % set the number of plot links
                n = 8 ;
                
                % generate plot data for each link
                link_verts = cell(1,n) ;
                for idx = 1:n
                    link_verts{idx} = (R{idx}*A.link_CAD_data{idx}.vertices' + ...
                        T{idx})' ;
                end
                
                if check_if_plot_is_available(A,'links')
                    for idx = 1:n
                        A.plot_data.links(idx).Faces = A.link_CAD_data{idx}.faces ;
                        A.plot_data.links(idx).Vertices = link_verts{idx} ;
                    end
                else
                    link_array = [] ;
                    for idx = 1:n
                        link_data = patch('Faces',A.link_CAD_data{idx}.faces,...
                            'Vertices',link_verts{idx},...
                            'FaceColor',A.link_plot_face_color,...
                            'FaceAlpha',A.link_plot_face_opacity,...
                            'EdgeColor',A.link_plot_edge_color,...
                            'LineWidth',A.link_plot_edge_width,...
                            'EdgeAlpha',A.link_plot_edge_opacity) ;
                        link_array = [link_array, link_data] ;
                    end
                    A.plot_data.links = link_array ;
                    
                    % turn camlight on
                    camlight
                end
            else
                plot_links@robot_arm_agent(A,time_or_config) ;
            end
        end
        
        %% custom axis limits
        function lims = get_axis_lims(A)
            % figure out the maximum length of the arm
            L = sum(A.link_sizes(1,:)) ;
            
            % create axis limits
            switch A.dimension
                case 2
                    lims = [-L,L,0,L] ;
                case 3
                    %                     switch A.floor_normal_axis
                    %                         case 1
                    %                             lims = [-L, L, -L, L, -L, L] ;
                    %                         case 2
                    %                             lims = [-L, L, 0, L, -L, L] ;
                    %                         case 3
                    %                             lims = [-L,L,-L,L,0,L] ;
                    %                     end
                    lims = [-0.5*L, L, -L, L, -L, 0.75*L] ;
                    
                    % in case base of robot is not a [0;0;0]:
                    lims = lims + [A.joint_locations(1, 1)*ones(1, 2), A.joint_locations(2, 1)*ones(1, 2), A.joint_locations(3, 1)*ones(1, 2)];
                    
                    % make z = 0 the ground
                    lims(5) = 0;
            end
        end
    end
end