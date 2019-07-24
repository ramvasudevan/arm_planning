classdef robot_arm_agent < multi_link_agent
    properties
    %% links
        % default arm is 2-D, 2-link, 2-DOF
        dimension = 2 ;
        n_links = 2 ;

        link_types = ['R','R'] ; % use 'V' for virtual links
        
        link_sizes = [0.55, 0.30 ;  % length
                      0.05, 0.05] ; % width
                  
        link_masses = (1e-03)*[0.15, 0.09] ; % ~ 2cm wide aluminum links

    %% joints
        % note that each link can only be associated with one joint
        % preceding it in the current forward kinematics formulation - so,
        % to make compound joints, one should make virtual links of zero
        % size
        
        n_joints = 2 ;
        joint_types = ['R','R'] ; % R for revolute, P for prismatic
        
        % the joint axes are in the predecessor's coordinate frame; for a
        % 2-D arm, revolute joints should use the z-axis, which is the
        % default here
        joint_axes = [0 0 ;
                      0 0 ;
                      1 1 ] ;
        
        % specify the location of each joint as an (x,y) coordinate in the
        % local frame of the pred/succ links; this array must have as many
        % columns as there are joints
        joint_locations = [+0.000 +0.250 ;   % predecessor x
                           +0.050 +0.000 ;   % predecessor y
                           -0.250 -0.125 ;   % successor x
                            0.000 +0.000 ] ; % successor y
        
        % joint limits and speed limits, where each column corresponds to
        % one joint; the top row is the minimum and the bottom is the
        % maximum, in rad or rad/s respectively
        joint_limits = [ +0, -Inf ;
                        +pi, +Inf] ;
                    
        joint_speed_limits = [-pi, -pi ;
                              +pi, +pi] ;
                        
        % state index data
        joint_state_indices
        joint_speed_indices

        % specify the kinematic chain of predecessor and successor links;
        % each column corresponds to a joint, the first row is the joint
        % predecessor link, and the second row is the successor link; 0
        % indicates the baselink
        kinematic_chain = [0, 1 ;   % predecessor index
                           1, 2 ] ; % successor index
                       
    %% physics
        % whether or not to simulate gravity
        gravity_flag = false ; % not implemented yet
        gravity_direction = [0 ; -1 ; 0] ;
    
        % torque limits; each column corresponds to one joint, the first
        % row is the minimum, and the second row is the maximum in N*m
        link_mass_torque_flag = false ; % not implemented yet
        
        joint_input_limits = 10.*[-1, -1 ;   % minimum
                                  +1, +1 ] ; % maximum
                               
    %% miscellaneous
        % integration
        integrator_time_discretization = 0.01 ; % s
    
        % collision checking
        collision_check_patch_data
        
        % plotting
        link_plot_data
        link_plot_face_color = [0 0 1] ;
        link_plot_face_opacity = 0.1 ;
        link_plot_edge_color = [0 0 1] ;
        link_plot_edge_opacity = 1.0 ;
        link_plot_edge_width = 1.25 ;
    end
    
    methods
    %% constructor
        function A = robot_arm_agent(varargin)
            % A = robot_arm_agent('property1',value1,'property2',value2,...)
            %
            % This class implements a generic multilink robot arm. By
            % default, it is a 2-D, 2-link, 2-DOF arm.
            
            % by default, the arm's states are (position,speed) of each
            % joint, in order from the first joint onwards
            n_states = 2*A.n_joints ;
            joint_state_indices = 1:2:n_states ;
            joint_speed_indices = 2:2:n_states ;
            
            n_inputs = A.n_joints ; % inputs are joint torques by default
            
            % default low-level controller
            LLC = robot_arm_PID_LLC() ;
            
            % create agent
            A = parse_args(A,'n_states',n_states,'n_inputs',n_inputs,...
                'joint_state_indices',joint_state_indices,...
                'joint_speed_indices',joint_speed_indices,...
                'LLC',LLC,...
                'set_axes_when_animating',true,...
                'animation_default_filename','arm_animation.gif',...
                'position_indices',[],varargin{:}) ;
            
            A.reset() ;
            A.LLC.setup(A)
            
            % set up plotting and collision check data
            A.create_plot_patch_data() ;
            A.create_collision_check_patch_data() ;
        end
        
        function create_plot_patch_data(A)
            % A.create_plot_patch_data()
            %
            % This method fills in A.link_plot_data with a cell array of
            % faces vectors (each 1-by-NF) and a cell array of vertices
            % arrays (each NV-by-2).
            
            % set up cell array to save patch data
            plot_faces_cell = cell(1,A.n_links) ;
            plot_verts_cell = cell(1,A.n_links) ;
            
            % create baselink triangle for plotting
            baselink_vertices = 0.05.*[-1, 1, 0 ;
                                        0, 0, 1 ]' ;
            baselink_faces = [1 2 3 1] ;
            
            % create links as ovals
            L = A.link_sizes ;
            
            for lidx = 1:size(L,2)
                % create box for link
                [link_faces, link_vertices] = make_oval(L(:,lidx)) ;
                
                % fill in cell array
                plot_faces_cell{lidx} = link_faces ;
                plot_verts_cell{lidx} = link_vertices ;
            end
            
            % fill in plot link data object
            A.link_plot_data.link_faces = plot_faces_cell ;
            A.link_plot_data.link_vertices = plot_verts_cell ;
            A.link_plot_data.baselink_faces = baselink_faces ;
            A.link_plot_data.baselink_vertices = baselink_vertices ;
        end
        
        function create_collision_check_patch_data(A)
            % A.create_collision_check_patch_data()
            %
            % Create a cell array of faces vectors (each 1-by-NF) and a
            % cell array of vertices arrays (each NV-by-2) to be used for
            % collision checking.
            
            % set up cell array to save patch data
            cc_faces_cell = cell(1,A.n_links) ;
            cc_verts_cell = cell(1,A.n_links) ;
            
            % create links as rectangles
            L = A.link_sizes ;
            
            for lidx = 1:size(L,2)
                % create box for link
                [link_faces, link_vertices] = make_box(L(:,lidx)) ;
                
                % fill in cell array
                cc_faces_cell{lidx} = link_faces ;
                cc_verts_cell{lidx} = link_vertices ;
            end
            
            % fill in collision check data object
            A.collision_check_patch_data.faces = cc_faces_cell ;
            A.collision_check_patch_data.vertices = cc_verts_cell ;
        end
        
    %% reset
        function reset(A,state)
            
            A.vdisp('Resetting states to 0',3) ;
            if nargin < 2
                A.state = zeros(A.n_states,1) ;
            else
                if length(state) ~= A.n_states
                    error('Input has incorrect number of states!')
                else
                    A.state = state(:) ;
                end
            end
            
            A.vdisp('Resetting time and inputs to 0',3)
            A.time = 0 ;
            A.input = zeros(A.n_inputs,1) ;
            A.input_time = 0 ;
            
            % reset LLC
            if isa(A.LLC,'arm_PID_LLC')
                A.vdisp('Resetting low-level controller integrator error.',3)
                A.LLC.position_error_state = zeros(length(A.joint_state_indices),1) ;
            end
        end
        
    %% get agent info
        function agent_info = get_agent_info(A)
            agent_info = get_agent_info@agent(A) ;
            agent_info.joint_limits = A.joint_limits ;
            agent_info.joint_speed_limits = A.joint_speed_limits ;
            agent_info.get_collision_check_volume = @(q) A.get_collision_check_volume(q) ;
            agent_info.collision_check_patch_data = A.collision_check_patch_data ;
            agent_info.forward_kinematics = @(t_or_q) A.forward_kinematics(t_or_q) ;
            agent_info.reach_limits = A.get_axis_lims() ;
        end
        
    %% create collision check volume
        function out = get_collision_check_volume(A,q)
            % out_volume = A.get_collision_check_volume(q)
            %
            % Given a configuration q...
            % If the agent is 2-D, return a polyline representing the
            % agent at that configuration. This can be used with polyxpoly
            % for collision checking.
            %
            % If the agent is 3-D, return a
            % patch structure with two fields (faces and vertices) that can
            % be used with SurfaceIntersection for collision checking.
            
            if nargin < 2
                q = A.state(A.joint_state_indices,1) ;
            end
            
            [R,T] = A.forward_kinematics(q) ;
            
            F_cell = A.collision_check_patch_data.faces ;
            V_cell = A.collision_check_patch_data.vertices ;
            
            switch A.dimension
                case 2
                    V = [] ;
                    for idx = 1:length(F_cell)
                        V_idx = R{idx}*V_cell{idx}' + T{idx} ;
                        V = [V, nan(2,1), V_idx(:,F_cell{idx})] ;
                    end
                    
                    out = V(:,2:end) ;
                case 3
                    F = [] ;
                    V = [] ;

                    N_verts = 0 ;
                    
                    for idx = 1:length(F_cell)
                        F_idx = F_cell{idx} + N_verts ;
                        V_idx = (R{idx}*V_cell{idx}' + T{idx})' ;
                        
                        F = [F ; F_idx] ;
                        V = [V ; V_idx] ;
                        
                        N_verts = size(V,1) ;
                    end
                    
                    out.faces = F ;
                    out.vertices = V ;
            end
        end

    %% stop

    %% forward kinematics
        function [R,T] = forward_kinematics(A,time_or_config)
            % [R,T] = A.forward_kinematics(time)
            % [R,T] = A.forward_kinematics(configuration)
            %
            % Compute the rotation and translation of all links in the
            % global (baselink) frame at the given time. If no time is
            % given, then it defaults to 0.
            
            if nargin < 2
                time_or_config = 0 ;
            end
            
            % get joint data
            if length(time_or_config) == 1
                t = time_or_config ;
                if t > A.time(end)
                    t = A.time(end) ;
                    warning(['Invalid time entered! Using agent''s final ',...
                        'time t = ',num2str(t),' instead.'])
                end
                
                % interpolate the state for the corresponding time
                z = match_trajectories(t,A.time,A.state) ;
                j_vals = z(1:2:end) ; % joint values                    
            else
                % assume a configuration was put in
                q = time_or_config ;
                
                if length(q) == A.n_states
                    q = q(1:2:end) ;
                elseif length(q) ~= A.n_states/2
                    error('Please provide either a time or a joint configuration.')
                end
                j_vals = q ;
            end
            j_locs = A.joint_locations ; % joint locations
            
            % extract dimensions
            n = A.n_links ;
            d = A.dimension ;
            
            % allocate cell arrays for the rotations and translations
            R = mat2cell(repmat(eye(d),1,n),d,d*ones(1,n)) ;
            T = mat2cell(repmat(zeros(d,1),1,n),d,ones(1,n)) ;
            
            % move through the kinematic chain and get the rotations and
            % translation of each link
            for idx = 1:A.n_joints
                k_idx = A.kinematic_chain(:,idx) ;
                p_idx = k_idx(1) ;
                s_idx = k_idx(2) ;
                
                % get the rotation and translation of the predecessor and
                % successor links; we assume the baselink is always rotated
                % to an angle of 0 and with a translation of 0
                if p_idx == 0
                    R_pred = eye(d) ;
                    T_pred = zeros(d,1) ;
                else
                    R_pred = R{p_idx} ;
                    T_pred = T{p_idx} ;
                end
                
                % get the value and location of the current joint
                j_idx = j_vals(idx) ;
                j_loc = j_locs(:,idx) ;
                
                % depending on the joint type, compute the rotation and
                % translation of the next link
                switch A.joint_types(idx)
                    case 'R'
                        if d == 3
                            % rotation matrix of current joint
                            axis_pred = R_pred*A.joint_axes(:,idx) ;
                            R_succ = axang2rotm([axis_pred', j_idx])*R_pred ;
                            
                            % location of current joint in global coords
                            T_succ = T_pred + R_pred*j_loc(1:3) - R_succ*j_loc(4:6) ;
                        else
                            % rotation matrix of current joint
                            R_succ = rotation_matrix_2D(j_idx)*R_pred ;
                            
                            % location of current joint in global coords
                            T_succ = T_pred + R_pred*j_loc(1:2) - R_succ*j_loc(3:4) ;
                        end
                    case 'P'
                        error('Prismatic joints are not supported yet!')
                    otherwise
                        error('Invalid joint type!')
                end
                
                % fill in rotation and translation cells
                R{s_idx} = R_succ ;
                T{s_idx} = T_succ ;
            end
        end
        
    %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % get desired torques and bound them
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            for idx = 1:length(u)
                u(idx) = bound_values(u(idx), A.joint_input_limits(:,idx)')  ;
            end
            
            % get the current speeds and bound them
            s = z(A.joint_speed_indices) ;
            for idx = 1:length(s)
                s(idx) = bound_values(s(idx), A.joint_speed_limits(:,idx)') ;
            end
            
            % get the torque exerted by the link masses on each joint
            if A.link_mass_torque_flag
                error('Link mass torque is not yet implemented!')
            end
            
            % compute dynamics
            zd = zeros(A.n_states,1) ;
            zd(A.joint_state_indices) = s(:) ;
            zd(A.joint_speed_indices) = u(:) ;
        end
        
    %% integrator
        function [t_out,z_out] = integrator(A,arm_dyn,t_span,z0)
            % [tout,zout] = A.integrator(arm_dynamics,tspan,z0)
            %
            % RK4 integration with joint limits and speed limits enforced.
            
            % create time vector
            dt = A.integrator_time_discretization ;
            t_out = t_span(1):dt:t_span(end) ;
            if t_out(end) < t_span(end)
                t_out = [t_out, t_span(end)] ;
            end
            
            % preallocate trajectory output
            Nt = size(t_out,2) ;
            z_out = [z0(:), nan(A.n_states,Nt-1)] ;

            % run integration loop
            for tidx = 2:Nt
                % get previous state
                z_cur = z_out(:,tidx-1) ;
                t_cur = t_out(tidx-1) ;
                
                % compute RK4 terms
                k1 = arm_dyn(t_cur, z_cur) ;
                k2 = arm_dyn(t_cur + dt/2, z_cur + dt*k1/2) ;
                k3 = arm_dyn(t_cur + dt/2, z_cur + dt*k2/2) ;
                k4 = arm_dyn(t_cur + dt, z_cur + dt*k3) ;
                
                % compute summed term
                dzdt = (1/6)*(k1 + 2*k2 + 2*k3 + k4) ;
                
                % compute next state
                z_new = z_cur + dt*dzdt ;
                
                % apply state limits
                joint_values = z_new(A.joint_state_indices)' ;
                joint_values = max([joint_values ; A.joint_limits(1,:)],[],1) ;
                joint_values = min([joint_values ; A.joint_limits(2,:)],[],1) ;
                z_new(A.joint_state_indices) = joint_values ;
                
                % apply speed limits
                joint_speeds = z_new(A.joint_speed_indices)' ;
                joint_speeds = max([joint_speeds; A.joint_speed_limits(1,:)],[],1) ;
                joint_speeds = min([joint_speeds; A.joint_speed_limits(2,:)],[],1) ;
                z_new(A.joint_speed_indices) = joint_speeds ;
                
                % save new state
                z_out(:,tidx) = z_new(:) ;
            end
        end
        
    %% plotting
        function plot(A,~)
            A.plot_at_time(A.time(end)) ;
        end
        
        function plot_at_time(A,t,~)
            if nargin < 2
                t = 0 ;
            end
            
            % plot baselink
            BF = A.link_plot_data.baselink_faces ;
            BV = A.link_plot_data.baselink_vertices ;
            
            if check_if_plot_is_available(A,'baselink')
                A.plot_data.baselink.Faces = BF ;
                A.plot_data.baselink.Vertices = BV ;
            else
                baselink_data = patch('Faces',BF,'Vertices',BV,...
                    'FaceColor',A.link_plot_face_color,...
                    'FaceAlpha',A.link_plot_face_opacity,...
                    'EdgeColor',A.link_plot_edge_color,...
                    'LineWidth',A.link_plot_edge_width,...
                    'EdgeAlpha',A.link_plot_edge_opacity) ;
                A.plot_data.baselink = baselink_data ;
            end
            
            % get the rotations and translations at the current time
            [R,T] = A.forward_kinematics(t) ;
            
            % generate plot data for each link
            link_verts = cell(1,A.n_links) ;
            for idx = 1:A.n_links
                link_verts{idx} = (R{idx}*A.link_plot_data.link_vertices{idx}' + ...
                                  T{idx})' ;
            end
            
            if check_if_plot_is_available(A,'links')
                for idx = 1:A.n_links
                    A.plot_data.links(idx).Faces = A.link_plot_data.link_faces{idx} ;
                    A.plot_data.links(idx).Vertices = link_verts{idx} ;
                end
            else
                link_array = [] ;
                for idx = 1:A.n_links
                    link_data = patch('Faces',A.link_plot_data.link_faces{idx},...
                        'Vertices',link_verts{idx},...
                        'FaceColor',A.link_plot_face_color,...
                        'FaceAlpha',A.link_plot_face_opacity,...
                        'EdgeColor',A.link_plot_edge_color,...
                        'LineWidth',A.link_plot_edge_width,...
                        'EdgeAlpha',A.link_plot_edge_opacity) ;
                    link_array = [link_array, link_data] ;
                end
                A.plot_data.links = link_array ;
            end
        end
        
        function lims = get_axis_lims(A)
            % figure out the maximum length of the arm
            L = 1.2*sum(A.link_sizes(1,:)) ;
            
            % create axis limits
            switch A.dimension
                case 2
                    lims = [-L,L,0,L] ;
                case 3
                    lims = [-L,L,-L,L,0,L] ;
            end
        end
    end
end