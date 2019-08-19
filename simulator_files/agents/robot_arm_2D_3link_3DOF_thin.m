classdef robot_arm_2D_3link_3DOF_thin < robot_arm_agent
    methods
        function A = robot_arm_2D_3link_3DOF_thin(varargin)
            n_links_and_joints = 3 ;
            
            dimension = 2 ;
            
            link_shapes = {'oval','oval','oval'} ;
            
            link_sizes = [1, 1, 1 ;
                          0.001, 0.001, 0.001] ;

            joint_locations = [+0.000 +0.500 +0.500 ;
                               +0.001 +0.000 +0.000 ;
                               -0.500 -0.500 -0.500 ;
                                0.000 +0.000 +0.000] ;
            
            joint_state_limits = [-Inf, -Inf, -Inf ;
                                  +Inf, +Inf, +Inf] ;
            
            joint_speed_limits = [-pi/2, -pi/2, -pi/2;
                                  +pi/2, +pi/2, +pi/2] ;
            
            joint_input_limits = 10.*[-1, -1, -1 ;
                                      +1, +1, +1 ] ;
            
            A@robot_arm_agent(varargin{:},...
                'n_links_and_joints',n_links_and_joints,...
                'dimension',dimension,...
                'link_shapes',link_shapes,...
                'link_sizes',link_sizes,...
                'joint_locations',joint_locations,...
                'joint_state_limits',joint_state_limits,...
                'joint_speed_limits',joint_speed_limits,...
                'joint_input_limits',joint_input_limits) ;
        end
    end
end