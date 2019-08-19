classdef robot_arm_2D_4link_4DOF < robot_arm_agent
    methods
        function A = robot_arm_2D_4link_4DOF(varargin)
            n_links_and_joints = 4 ;
            
            dimension = 2 ;
            
            link_shapes = {'oval','oval','oval','oval'} ;
            
            link_sizes = [0.20, 0.20, 0.20, 0.20 ;
                          0.04, 0.04, 0.04, 0.04] ;

            joint_locations = [+0.000 +0.075 +0.075 +0.075 ;
                               +0.050 +0.000 +0.000 +0.000 ;
                               -0.075 -0.075 -0.075 +0.075 ;
                                0.000 +0.000 +0.000 +0.000] ;
            
            joint_state_limits = [+0,  -pi, -pi, -pi ;
                                  +pi, +pi, +pi, +pi] ;
            
            joint_speed_limits = repmat([-pi ; +pi],1,4) ;
            
            joint_input_limits = repmat([-10;10],1,4) ;
            
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