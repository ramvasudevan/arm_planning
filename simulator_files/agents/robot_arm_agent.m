classdef robot_arm_agent < multi_link_agent
    properties
        %% links
        % the number of links and joints of the arm; we use the convention
        % that every joint is succeeded by a link (and every link except
        % the baselink is preceded by a joint), hence the number of joints
        % and links are the same
        n_links_and_joints
        
        % pick link shapes as a cell array containing the strings:
        % 'box' (2D), 'oval' (2D), 'cuboid' (3D), or ellipsoid (3D)
        % (note that the baselink is plotted as a triangle in 2D and a cone
        % in 3D by default)
        link_shapes
        
        % link sizes is a 2-by-n_links_and_joints array, where each row
        % specifies the breadth of the link along the x, y, and z
        % dimensions (note that, for ellipsoid-shaped links, these values
        % specify the diameters of the ellipsoid along the link's local x,
        % y, and z dimensions)
        link_sizes
        
        % the links all have 1 kg mass by default
        link_masses
        
        %% joints
        % note that each link can only be associated with one joint
        % preceding it in the current forward kinematics formulation
        
        % joint types are a cell array containing the strings 'revolute'
        % and 'prismatic' of size 1-by-n_links_and_joints; note
        joint_types
        
        % the joint axes are in the predecessor's coordinate frame; for a
        % 2-D arm, revolute joints should use the z-axis, which is the
        % default here
        joint_axes
        
        % specify the location of each joint as an (x,y) coordinate in the
        % local frame of the pred/succ links; this array must have as many
        % columns as there are joints
        joint_locations
        
        % joint limits and speed limits, where each column corresponds to
        % one joint; the top row is the minimum and the bottom is the
        % maximum, in rad or rad/s respectively
        joint_state_limits
        joint_speed_limits
        
        % state index data
        joint_state_indices
        joint_speed_indices
        
        % specify the kinematic chain of predecessor and successor links;
        % each column corresponds to a joint, the first row is the joint
        % predecessor link, and the second row is the successor link; 0
        % indicates the baselink
        kinematic_chain
        
        %% physics
        % whether or not to simulate gravity
        gravity_flag = false ; % not implemented yet
        gravity_direction
        
        % torque limits; each column corresponds to one joint, the first
        % row is the minimum, and the second row is the maximum in N*m
        joint_input_limits
        
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
            
            % make sure there are input arguments
            if nargin == 0
                error(['Please create a subclass of this class with all ',...
                    'required properties defined!'])
            end
            
            % by default, the arm's states are (position,speed) of each
            % joint, in order from the first joint onwards
            n_states = 2*A.n_links_and_joints ;
            joint_state_indices = 1:2:n_states ;
            joint_speed_indices = 2:2:n_states ;
            
            n_inputs = A.n_links_and_joints ; % inputs are joint torques by default
            
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
            
            % ensure properties are consistent
            A.check_and_fix_properties() ;
            
            % set time, states, and inputs to zero
            A.reset() ;
            
            % set up low-level controller
            A.LLC.setup(A)
            
            % set up plotting and collision check data
            A.create_plot_patch_data() ;
            A.create_collision_check_patch_data() ;
        end
        
        function check_and_fix_properties(A)
            % A.check_and_fix_properties()
            %
            % Go through each of the arm's default properties and make sure
            % they are consisten with the total number of links and joints,
            % and with the dimension of the arm's workspace.
            
            A.vdisp('Ensuring all arm properties are consistent',3)
            
            % get the actual number of links and joints; the number of
            % links MUST be equal to the number of joints, because every
            % joint is connected to a single predecessor and a single
            % successor link
            N = A.n_links_and_joints ;
            
            % check dimension
            d = A.dimension ;
            if (d ~= 2) && (d ~= 3)
                error(['The arm''s workspace dimension (',num2str(d),...
                    ') is incorrect! Pick 2 or 3.'])
            end
            
            %% states and state indices
            A.vdisp('Checking states and state indices',5)
            
            % by default, the arm's states are (position,speed) of each
            % joint
            if isempty(A.n_states)
                A.n_states = 2*N ;
                A.vdisp(['Setting number of states to ',num2str(A.n_states)],6)
            end
            
            if isempty(A.joint_state_indices)
                A.joint_state_indices = 1:2:A.n_states ;
                A.vdisp('Setting default joint state indices',6)
            end
            
            if isempty(A.joint_speed_indices)
                A.joint_speed_indices = 2:2:A.n_states ;
                A.vdisp('Setting default joint speed indices',6)
            end
            
            if isempty(A.n_inputs)
                A.n_inputs = 2*N ;
                A.vdisp(['Setting number of inputs to ',num2str(A.n_states)],6)
            end
            
            %% links
            A.vdisp('Checking links',5)
            
            % check dimension and number of links
            [d_l, N_l] = size(A.link_sizes) ;
            
            if N_l ~= N
                error(['The arm does not have the same number of links as ',...
                    'its n_links_and_joints property!'])
            end
            
            if d_l ~= d
                error(['The arm''s links are not of the same dimension (',...
                    num2str(d_l),' as the arm''s dimension property (',...
                    num2str(d),')!'])
            end
            
            % check link shapes
            N_link_shapes = length(A.link_shapes) ;
            if N_link_shapes > N
                A.vdisp('Removing extra link shapes',6)
                A.link_shapes = A.link_shapes(1:N) ;
            elseif N_link_shapes < N
                A.vdisp('Setting missing link shapes',6)
                switch d
                    case 2
                        A.link_shapes = [A.link_shapes, repmat({'box'},1,N-N_link_shapes)] ;
                    case 3
                        A.link_shapes = [A.link_shapes, repmat({'cuboid'},1,N-N_link_shapes)] ;
                end
            end
            
            % check link masses
            if isempty(A.link_masses) || length(A.link_masses) ~= N
                A.vdisp('Setting link masses',6)
                A.link_masses = ones(1,N) ;
            end
            
            %% joints
            A.vdisp('Checking joints',5)
            
            % check dimension and number of joints
            [d_j_2x, N_j] = size(A.joint_locations) ;
            d_j = d_j_2x / 2 ;
            
            if N_j ~= N
                error(['The arm does not have the same number of joints as ',...
                    'its n_links_and_joints property!'])
            end

            if d_j ~= d
                error(['The arm''s joints are not of the same dimension (',...
                    num2str(d_j),') as the arm''s dimension property (',...
                    num2str(d),')!'])
            end
            
            % set default joint types (all revolute)
            if isempty(A.joint_types) || length(A.joint_types) ~=N
                A.joint_types = repmat({'revolute'},1,N) ;
            end
            
            % set default joint axes (note that these axes are in the
            % local coordinate frame of the joint's predecessor link)
            N_joint_axes = size(A.joint_axes,2) ;
            if N_joint_axes > N
                A.vdisp('Removing extra joint axes',6)
                A.joint_axes = A.joint_axes(:,1:N) ;
            elseif N_joint_axes < N
                A.vdisp('Setting missing joint axes',6)
                A.joint_axes = [A.joint_axes, repmat([0;0;1],1,N-N_joint_axes)] ;
            end
            
            if isempty(A.kinematic_chain)
                A.vdisp('Setting arm kinematic chain',6)
                A.kinematic_chain = [0:(N-1) ; 1:N] ;
            end
            
            %% physics
            if isempty(A.gravity_direction)
                A.vdisp('Setting default gravity direction',9)
                switch d
                    case 2
                        A.gravity_direction = [0;-1;0] ;
                    case 3
                        A.gravity_direction = [0;0;-1] ;
                end
            end
            
        end
        
        %% plot data
        function create_plot_patch_data(A)
            % A.create_plot_patch_data()
            %
            % This method fills in A.link_plot_data with a cell array of
            % faces vectors (each 1-by-NF) and a cell array of vertices
            % arrays (each NV-by-2).
            
            % set up cell array to save patch data
            plot_faces_cell = cell(1,A.n_links_and_joints) ;
            plot_verts_cell = cell(1,A.n_links_and_joints) ;
            
            % get link sizes
            L = A.link_sizes ;
            
            switch A.dimension
                case 2
                    % create baselink triangle for plotting
                    baselink_vertices = 0.05.*[-1, 1, 0 ;
                        0, 0, 1 ]' ;
                    baselink_faces = [1 2 3 1] ;
                    
                case 3
                    % create baselink cone for plotting
                    [baselink_faces,baselink_vertices] = make_cone_for_patch(0.05,0.05) ;
            end
                    
            % create links
            for l_idx = 1:size(L,2)
                l = L(:,l_idx) ;
                
                % create link based on link type
                switch A.link_shapes{l_idx}
                    case 'box'
                        [link_faces, link_vertices] = make_box(l) ;
                    case 'oval'
                        [link_faces, link_vertices] = make_oval(l) ;
                    case 'cuboid'
                        % create box for link that is slightly shorter than
                        % the actual volume of the box for prettiness
                        % purposes
                        l(1) = l(1) - 0.045 ;
                        [link_faces,link_vertices] = make_cuboid_for_patch(l) ;
                    case 'ellipsoid'
                        % remember that the link sizes specify the
                        % diameters of the link in each dimension
                        l = l./2 ;
                        [link_faces,link_vertices] = make_ellipsoid_for_patch(l(1),l(2),l(3),zeros(3,1),6) ;
                    otherwise
                        error('Invalid link type! Pick cuboid or ellipsoid.')
                end

                % fill in cell array
                plot_faces_cell{l_idx} = link_faces ;
                plot_verts_cell{l_idx} = link_vertices ;
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
            cc_faces_cell = cell(1,A.n_links_and_joints) ;
            cc_verts_cell = cell(1,A.n_links_and_joints) ;
            
            % create links as rectangles
            L = A.link_sizes ;
            
            for l_idx = 1:size(L,2)
                l = L(:,l_idx) ;
                
                % create link based on link type
                switch A.dimension
                    case 2
                        [link_faces, link_vertices] = make_box(l) ;
                    case 3
                        switch A.link_shapes{l_idx}
                            case 'cuboid'
                                [~,link_vertices] = make_cuboid_for_patch(l) ;
                            case 'ellipsoid'
                                [~,link_vertices] = make_ellipsoid_for_patch(l(1),l(2),l(3),zeros(3,1),6) ;
                        end
                        link_faces = convhull(link_vertices) ;
                end
                
                % fill in cell array
                cc_faces_cell{l_idx} = link_faces ;
                cc_verts_cell{l_idx} = link_vertices ;
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
            agent_info.joint_state_indices = A.joint_state_indices ;
            agent_info.joint_speed_indices = A.joint_speed_indices ;
            agent_info.joint_state_limits = A.joint_state_limits ;
            agent_info.joint_speed_limits = A.joint_speed_limits ;
            agent_info.get_collision_check_volume = @(q) A.get_collision_check_volume(q) ;
            agent_info.collision_check_patch_data = A.collision_check_patch_data ;
            agent_info.forward_kinematics = @(t_or_q) A.forward_kinematics(t_or_q) ;
            agent_info.reach_limits = A.get_axis_lims() ;
        end
        
        %% get collision check volume
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
            n = A.n_links_and_joints ;
            d = A.dimension ;
            
            % allocate cell arrays for the rotations and translations
            R = mat2cell(repmat(eye(d),1,n),d,d*ones(1,n)) ;
            T = mat2cell(repmat(zeros(d,1),1,n),d,ones(1,n)) ;
            
            % move through the kinematic chain and get the rotations and
            % translation of each link
            for idx = 1:A.n_links_and_joints
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
                switch A.joint_types{idx}
                    case 'revolute'
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
                    case 'prismatic'
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
            N_t = size(t_out,2) ;
            z_out = [z0(:), nan(A.n_states,N_t-1)] ;
            
            % run integration loop
            for tidx = 2:N_t
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
                joint_values = max([joint_values ; A.joint_state_limits(1,:)],[],1) ;
                joint_values = min([joint_values ; A.joint_state_limits(2,:)],[],1) ;
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
            link_verts = cell(1,A.n_links_and_joints) ;
            for idx = 1:A.n_links_and_joints
                link_verts{idx} = (R{idx}*A.link_plot_data.link_vertices{idx}' + ...
                    T{idx})' ;
            end
            
            if check_if_plot_is_available(A,'links')
                for idx = 1:A.n_links_and_joints
                    A.plot_data.links(idx).Faces = A.link_plot_data.link_faces{idx} ;
                    A.plot_data.links(idx).Vertices = link_verts{idx} ;
                end
            else
                link_array = [] ;
                for idx = 1:A.n_links_and_joints
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
            L = sum(A.link_sizes(1,:)) ;
            
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