classdef robot_arm_3D_fetch < robot_arm_agent
    methods
        function A = robot_arm_3D_fetch(varargin)
            dimension = 3 ;
                      
            n_links_and_joints = 6 ;
            
            n_states = 2*n_links_and_joints ;
            
            link_shapes = {'cuboid', 'cuboid','cuboid','cuboid','cuboid', 'cuboid'} ;
            
%             link_sizes = [0.05, 0.300, 0.05, 0.200 ;  % size in x
%                           0.05, 0.025, 0.05, 0.025 ;  % size in y
%                           0.05, 0.025, 0.05, 0.025] ; % size in z

            buffer_dist = 0.15;

            link_sizes = [0.001, 0.330, 0.001, 0.330, 0.001, 0.330 ;  % size in x
                          0.001, 0.150, 0.001, 0.150, 0.001, 0.150 ;  % size in y
                          0.001, 0.150, 0.001, 0.150, 0.001, 0.150] ; % size in z
            
            joint_state_indices = 1:2:n_states ;
            
            joint_speed_indices = 2:2:n_states ;
            
            joint_types = {'revolute','revolute','revolute','revolute', 'revolute', 'revolute'} ;
            
            joint_axes = [0 0 1 0 1 0;
                          0 1 0 1 0 1;
                          1 0 0 0 0 0] ; % axes are in preceding link's frame
                      
            joint_locations = [+0.00, +0.00, +0.33/2, +0.00, +0.33/2, +0.00 ; % predecessor x
                               +0.00, +0.00, +0.00, +0.00, +0.00, +0.00 ; % predecessor y
                               +0.00, +0.00, +0.00, +0.00, +0.00, +0.00 ; % predecessor z
                               +0.00, -0.33/2, +0.00, -0.33/2, +0.00, -0.33/2 ; % successor x
                               +0.00, +0.00, +0.00, +0.00, +0.00, +0.00 ; % successor y
                               +0.00, +0.00, +0.00, +0.00, +0.00, +0.00 ];% successor z
                           
            joint_state_limits = [-Inf, -Inf, -Inf, -Inf, -Inf, -Inf ;
                            +Inf, +Inf, +Inf, +Inf, +Inf, +Inf ] ;
                        
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
                varargin{:}) ;
        end
        
        function [faces,vertices] = create_baselink_plot_patch_data(A)
            % create baselink cone for plotting
            [faces,vertices] = make_cuboid_for_patch(0.025, 0.025, 0.025, [0;0;0]) ;
        end
    end
end