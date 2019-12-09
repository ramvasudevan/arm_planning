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

            buffer_dist = 0.1460;

            link_sizes = [0.1206, 0.4635, 0.001, 0.4254, 0.001, 0.3810 ;  % size in x
                          0.1460, 0.1460, 0.001, 0.150, 0.001, 0.1460 ;  % size in y
                          0.0825, 0.1460, 0.001, 0.150, 0.001, 0.1460] ; % size in z
            
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
                varargin{:}) ;
        end
        
        function [faces,vertices] = create_baselink_plot_patch_data(A)
            % create baselink cone for plotting
            [faces,vertices] = make_cuboid_for_patch(0.025, 0.025, 0.025, [0;0;0]) ;
        end
        
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