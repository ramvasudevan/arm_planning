classdef robot_arm_3D_2link_4DOF < robot_arm_agent
    methods
        function A = robot_arm_3D_2link_4DOF(varargin)
            dimension = 3 ;
                      
            n_links_and_joints = 4 ;
            
            n_states = 2*n_links_and_joints ;
            
            link_shapes = {'ellipsoid','cuboid','ellipsoid','cuboid'} ;
            
            link_sizes = [0.05, 0.300, 0.05, 0.200 ;  % size in x
                          0.05, 0.025, 0.05, 0.025 ;  % size in y
                          0.05, 0.025, 0.05, 0.025] ; % size in z
            
            joint_state_indices = 1:2:n_states ;
            
            joint_speed_indices = 2:2:n_states ;
            
            joint_types = {'revolute','revolute','revolute','revolute'} ;
            
            joint_axes = [0 0 1 0 ;
                          0 1 0 1 ;
                          1 0 0 0 ] ; % axes are in preceding link's frame
                      
            joint_locations = [+0.00, +0.00, +0.15, +0.00 ; % predecessor x
                               +0.00, +0.00, +0.00, +0.00 ; % predecessor y
                               +0.05, +0.00, +0.00, +0.00 ; % predecessor z
                               +0.00, -0.15, +0.00, +0.10 ; % successor x
                               +0.00, +0.00, +0.00, +0.00 ; % successor y
                               +0.00, +0.00, +0.00, +0.00 ];% successor z
                           
            joint_state_limits = [-Inf, -pi, -Inf, -Inf ;
                            +Inf,   0, +Inf, +Inf] ;
                        
            joint_speed_limits = (pi/2).*repmat([-1;1],1,n_links_and_joints) ;
            
            joint_input_limits = 3.*joint_speed_limits;
            
            kinematic_chain = [0 1 2 3 ;
                               1 2 3 4] ;
                           
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
                'animation_view',animation_view) ;
        end
    end
end