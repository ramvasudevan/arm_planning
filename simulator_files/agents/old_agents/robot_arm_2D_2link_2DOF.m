classdef robot_arm_2D_2link_2DOF < robot_arm_agent
    methods
        function A = robot_arm_2D_2link_2DOF(varargin)
            n_links_and_joints = 2 ;
            
            dimension = 2 ;
            
            link_shapes = {'oval','oval','oval'} ;
            
            link_sizes = [0.55, 0.30 ;
                          0.05, 0.05] ;

            joint_locations = [+0.000 +0.250 ;
                               +0.050 +0.000 ;
                               -0.250 -0.125 ;
                                0.000 +0.000 ] ;
            
            joint_state_limits = [+0, -Inf ;
                                  +pi, +Inf] ;
            
            joint_speed_limits = [-pi, -pi ;
                                  +pi, +pi] ;
            
            joint_input_limits = 10.*[-1, -1 ;
                                      +1, +1 ] ;
            
            A@robot_arm_agent(varargin{:},...
                'n_links_and_joints',n_links_and_joints,...
                'dimension',dimension,...
                'link_shapes',link_shapes,...
                'link_sizes',link_sizes,...
                'joint_locations',joint_locations,...
                'joint_state_limits',joint_state_limits,...
                'joint_speed_limits',joint_speed_limits,...
                'joint_input_limits',joint_input_limits,...
                varargin{:}) ;
        end
    end
end