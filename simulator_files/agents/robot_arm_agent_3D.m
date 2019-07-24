classdef robot_arm_agent_3D < robot_arm_agent
    methods
        %% constructor
        function A = robot_arm_agent_3D(varargin)
            % A = robot_arm_agent_3D('property1',value1,...
            %                        'property2',value2,...)
            %
            % This class implements a 3-D, 4-DOF, 2-link robot arm.
            
            dimension = 3 ;
            
            n_links = 4 ; % 2 real links, 2 virtual links
                      
            n_joints = 4 ;
            
            n_states = 2*n_joints ;
            
            % 'V' for virtual links, used to define compound joints
            link_types = ['V','R','V','R'] ;
            
            % links 1 and 3 are virtual links that will be represented as
            % little spheres, so the three values for their sizes in x, y,
            % and z are the radii, respectively, of an axis-aligned
            % ellipsoid
            link_sizes = [0.025, 0.300, 0.025, 0.200 ;  % size in x
                          0.025, 0.025, 0.025, 0.025 ;  % size in y
                          0.025, 0.025, 0.025, 0.025] ; % size in z
            
            joint_state_indices = 1:2:n_states ;
            
            joint_speed_indices = 2:2:n_states ;
            
            joint_types = ['R','R','R','R'] ;
            
            joint_axes = [0 0 1 0 ;
                          0 1 0 1 ;
                          1 0 0 0 ] ; % axes are in preceding joint's frame
                      
            joint_locations = [+0.00, +0.00, +0.15, +0.00 ; % predecessor x
                               +0.00, +0.00, +0.00, +0.00 ; % predecessor y
                               +0.05, +0.00, +0.00, +0.00 ; % predecessor z
                               +0.00, -0.15, +0.00, -0.10 ; % successor x
                               +0.00, +0.00, +0.00, +0.00 ; % successor y
                               +0.00, +0.00, +0.00, +0.00 ];% successor z
                           
            joint_limits = [-Inf, -pi, -Inf, -Inf ;
                            +Inf,   0, +Inf, +Inf] ;
                        
            joint_speed_limits = (pi/2).*repmat([-1;1],1,n_joints) ;
            
            joint_input_limits = 3.*joint_speed_limits;
            
            kinematic_chain = [0 1 2 3 ;
                               1 2 3 4] ;
                           
            gravity_direction = [0;0;-1] ;
            
            set_view_when_animating = true ;
            
            animation_view = 3 ;
            
            A@robot_arm_agent('dimension',dimension,'n_links',n_links,...
                'n_joints',n_joints,'n_inputs',n_joints,...
                'n_states',n_states,...
                'link_types',link_types,'link_sizes',link_sizes,...
                'joint_state_indices',joint_state_indices,...
                'joint_speed_indices',joint_speed_indices,...
                'joint_types',joint_types,'joint_axes',joint_axes,...
                'joint_locations',joint_locations,...
                'joint_limits',joint_limits,...
                'joint_speed_limits',joint_speed_limits,...
                'joint_input_limits',joint_input_limits,...
                'kinematic_chain',kinematic_chain,...
                'gravity_direction',gravity_direction,...
                'set_view_when_animating',set_view_when_animating,...
                'animation_view',animation_view) ;
        end
        
        function create_plot_patch_data(A)
            % set up cell array to save patch data
            plot_faces_cell = cell(1,A.n_links) ;
            plot_verts_cell = cell(1,A.n_links) ;
            
            % create baselink cone for plotting
            [baselink_faces,baselink_vertices] = make_cone_for_patch(0.05,0.05) ;
            
            % create links as cuboids
            L = A.link_sizes ;
            
            for lidx = 1:size(L,2)
                l = L(:,lidx) ;
                switch A.link_types(lidx)
                    case 'R'
                        % create box for link that is slightly shorter than
                        % the actual volume of the box for prettiness
                        % purposes
                        l(1) = l(1) - 0.045 ;
                        [link_faces,link_vertices] = make_cuboid_for_patch(l) ;
                    case 'V'
                        % create ellipsoid for link
                        [link_faces,link_vertices] = make_ellipsoid_for_patch(l(1),l(2),l(3),zeros(3,1),6) ;
                    otherwise
                        error('Invalid link type! Pick V for virtual or R for real.')
                end
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
            % set up cell array to save patch data
            cc_faces_cell = cell(1,A.n_links) ;
            cc_verts_cell = cell(1,A.n_links) ;
            
            % create links as cuboids
            L = A.link_sizes ;
            
            for lidx = 1:size(L,2)
                l = L(:,lidx) ;
                switch A.link_types(lidx)
                    case 'R'
                        [~,link_vertices] = make_cuboid_for_patch(l) ;
                    case 'V'
                        % create ellipsoid for link
                        [~,link_vertices] = make_ellipsoid_for_patch(l(1),l(2),l(3),zeros(3,1),6) ;
                    otherwise
                        error('Invalid link type! Pick V for virtual or R for real.')
                end
                
                % get triangulated link faces for collision checking
                link_faces = convhull(link_vertices) ;
                
                % fill in cell array
                cc_faces_cell{lidx} = link_faces ;
                cc_verts_cell{lidx} = link_vertices ;
            end
            
            % fill in plot link data object
            A.collision_check_patch_data.faces = cc_faces_cell ;
            A.collision_check_patch_data.vertices = cc_verts_cell ;
        end
    end
end