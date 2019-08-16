classdef robot_arm_straight_line_HLP < high_level_planner
    methods
        %%  constructor
        function HLP = robot_arm_straight_line_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
        end
        
        %% setup
        function setup(HLP,agent_info,world_info)
            % get all the necessary arm properties filled in
            HLP.vdisp('Filling in HLP''s arm properties',9)
            HLP = fill_in_arm_properties(HLP,agent_info,true) ;
            
            % get the world goal
            HLP.vdisp('Setting goal',9)
            HLP.goal = world_info.goal ;
        end
        
        %% get waypoint
        function waypoint = get_waypoint(HLP,agent_info,~,lookahead_distance)
            q_cur = agent_info.state(HLP.arm_joint_state_indices, end) ;
            q_goal = HLP.goal ;
            dir_des = q_goal - q_cur ;
            dir_des = dir_des./norm(dir_des) ;
            if nargin < 4
                lookahead_distance = HLP.default_lookahead_distance ;
            end
            waypoint = q_cur + lookahead_distance.*dir_des ;
        end
    end
end