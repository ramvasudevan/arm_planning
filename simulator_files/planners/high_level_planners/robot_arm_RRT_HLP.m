classdef robot_arm_RRT_HLP < robot_arm_sampling_based_HLP
    %% properties
    properties
        make_new_tree_every_iteration_flag = false ;
        new_node_growth_distance = 0.1 ;
    end
    %% methods
    methods
        %% constructor
        function HLP = robot_arm_RRT_HLP(varargin)
            HLP@robot_arm_sampling_based_HLP(varargin{:}) ;
        end
        
        %% sampling
        function sample(HLP,agent_info,world_info)
            if HLP.make_new_tree_every_iteration_flag
                HLP.initialize_graph(agent_info,world_info)
            end
            
            % call superclass sample method
            sample@robot_arm_sampling_based_HLP(HLP,agent_info,world_info) ;
        end
        
        function q_new = create_new_node(HLP,~,~)
            HLP.vdisp('Creating new node',9)
            
            % create random node
            q_new = rand_range(HLP.arm_joint_state_limits(1,:),...
                               HLP.arm_joint_state_limits(2,:))' ;
                           
             % find nearest neighbor in tree
            q_near = HLP.find_nearest_node(q_new) ;

            % grow tree towards new node
            q_dir = q_new - q_near ;
            q_dir = q_dir./norm(q_dir) ;
            q_new = q_near + HLP.new_node_growth_distance.*q_dir ;
        end
    end
end