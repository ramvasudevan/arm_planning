classdef robot_arm_2D_3link_3DOF < robot_arm_agent
    methods
        function A = robot_arm_2D_3link_3DOF(varargin)
            n_links_and_joints = 3 ;
            
            dimension = 2 ;
            
            link_shapes = {'oval','oval','oval'} ;
            
            link_sizes = [0.20, 0.20, 0.20 ;
                          0.05, 0.05, 0.05] ;

            joint_locations = [+0.000 +0.075 +0.075 ;
                               +0.050 +0.000 +0.000 ;
                               -0.075 -0.075 -0.100 ;
                                0.000 +0.000 +0.000] ;
            
            joint_state_limits = [+0,  -Inf, -pi ;
                                  +pi, +Inf, +pi] ;
            
            joint_speed_limits = repmat([-pi ; +pi],1,3) ;
            
            joint_input_limits = repmat([-10;10],1,3) ;
            
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