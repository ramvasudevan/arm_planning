classdef robot_arm_2D_4link_4DOF_thin < robot_arm_agent
    methods
        function A = robot_arm_2D_4link_4DOF_thin(varargin)
            n_links_and_joints = 4 ;
            
            dimension = 2 ;
            
            link_shapes = {'oval','oval','oval','oval'} ;
            
            link_sizes = [1, 1, 1, 1 ;
                          0.001, 0.001, 0.001, 0.001 ] ;

            joint_locations = [0.000 0.5 0.5 0.5  ;
                               0.001 0 0 0 ;
                               -0.5 -0.5 -0.5 -0.5 ;
                                0 0 0 0 ] ;
            
%             joint_state_limits = [-Inf, -Inf, -Inf ;
%                                   +Inf, +Inf, +Inf] ;
            joint_state_limits = ones(2,4).*[-inf;inf];
            
%             joint_speed_limits = [-pi/2, -pi/2, -pi/2;
%                                   +pi/2, +pi/2, +pi/2] ;
            joint_speed_limits = ones(2,4).*[-pi/2;pi/2];

%             joint_input_limits = 10.*[-1, -1, -1 ;
%                                       +1, +1, +1 ] ;

            joint_input_limits = ones(2,4).*[-10;10];
            
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