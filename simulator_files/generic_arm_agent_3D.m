classdef generic_arm_agent_3D < generic_arm_agent
    methods
        %% constructor
        function A = generic_arm_agent_3D(varargin)
            % A = generic_arm_agent_3D('property1',value1,...
            %                          'property2',value2,...)
            %
            % This class implements a 3-D, 4-DOF, 2-link robot arm.
            
            dimension = 3 ;
            
            n_links = 2 ;
                      
            n_joints = 4 ;
            
            n_states = 2*n_joints ;
            
            link_sizes = [0.30, 0.20 ;
                          0.05, 0.05 ;
                          0.05, 0.05] ;
            
            joint_state_indices = 1:2:n_states ;
            
            joint_speed_indices = 2:2:n_states ;
            
            joint_types = ['R','R','R','R'] ;
            
            joint_axes = [0 0 1 0 ;
                          0 1 0 1 ;
                          1 0 0 0 ] ;
                      
            joint_locations = [+0.00, +0.00, +0.15, +0.15 ; % predecessor x
                               +0.00, +0.00, +0.00, +0.00 ; % predecessor y
                               +0.05, +0.05, +0.00, +0.00 ; % predecessor z
                               -0.15, -0.15, -0.10, -0.10 ; % successor x
                               +0.00, +0.00, +0.00, +0.00 ; % successor y
                               +0.00, +0.00, +0.00, +0.00 ];% successor z
                           
            joint_limits = [-Inf, 0,  -Inf, -Inf ;
                            +Inf, pi, +Inf, +Inf] ;
                        
            joint_speed_limits = (pi/2).*repmat([-1;1],1,n_joints) ;
            
            joint_input_limits = 3.*joint_speed_limits;
            
            kinematic_chain = [0 0 1 1 ;
                               1 1 2 2] ;
                           
            gravity_direction = [0;0;-1] ;
            
            A@generic_arm_agent('dimension',dimension,'n_links',n_links,...
                'n_joints',n_joints,'n_inputs',n_joints,...
                'n_states',n_states,'link_sizes',link_sizes,...
                'joint_state_indices',joint_state_indices,...
                'joint_speed_indices',joint_speed_indices,...
                'joint_types',joint_types,'joint_axes',joint_axes,...
                'joint_locations',joint_locations,...
                'joint_limits',joint_limits,...
                'joint_speed_limits',joint_speed_limits,...
                'joint_input_limits',joint_input_limits,...
                'kinematic_chain',kinematic_chain,...
                'gravity_direction',gravity_direction) ;
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
                % create box for link
                [link_faces,link_vertices] = make_cuboid_for_patch(L(:,lidx)) ;
                
                % fill in cell array
                plot_faces_cell{lidx} = link_faces ;
                plot_verts_cell{lidx} = link_vertices' ;
            end
            
            % fill in plot link data object
            A.plot_link_data.link_faces = plot_faces_cell ;
            A.plot_link_data.link_vertices = plot_verts_cell ;
            A.plot_link_data.baselink_faces = baselink_faces ;
            A.plot_link_data.baselink_vertices = baselink_vertices' ;
        end
        
        function create_collision_check_patch_data(A)
            % by default, use the same data as the plot
            A.create_plot_patch_data() ;
            A.collision_check_data.faces = A.plot_link_data.link_faces ;
            A.collision_check_data.vertices = A.plot_link_data.link_vertices ;
        end
    end
end