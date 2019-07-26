classdef robot_arm_RRT_HLP < high_level_planner
    % RRT in configuration space
    % Daphna Raz 2019
    
    properties
        
        % tree params
        k = 100 ;       % number of nodes
        nodes = [] ;     % (n_states/2) x k matrix of nodes
                             % Node defined as [q_1 q_2 .. q_n]
        node_parent_idx = [] % parent index corresponding to each node
                               % dimensions are dim x num_nodes
                               % start node parent = 0
                    
        d_edge = 0.1 ;       % distance to extend node (euclidean)
        
        
        % path
        path = [] % path of configuration from start to finish - should become part of waypoints vector?
        
    end
    
    methods
        %% constructor
        function HLP = robot_arm_RRT_HLP(varargin)
            
            HLP@high_level_planner(varargin{:}) ;
            
        end
        
        function q_rand = generate_random_config(HLP, bounds)
            % Generate a random configuration within bounds
            
            q_rand = zeros(HLP.dimension,1);
            
            % loop through bounds of each joint 
            % TODO make sure bounds are not inf
            for i = 1:HLP.dimension
                a = bounds(1,i);
                b = bounds(2,i);
                r_rand = (b-a).*rand(1,1) + a;
                q_rand(i) = r_rand;
            end
            
        end
        
        % TODO Double check that in MATLAB class function has access to its
        % members
        
        function q_near_idx = find_nearest_node(HLP, q_rand)
            
            % TODO may need to initialize node so I'm not looking in an
            % empty vector
            dim = HLP.dimension;
            q_near = HLP.nodes(1:dim,1)
            q_near_idx = 1
            % use l2 norm for now
            current_min_norm = norm(q_near - q_rand)
            
            % loop through every node
            for i = 1:size(HLP.nodes,2)
                %k = i
                test_norm = norm(HLP.nodes(1:dim,i) - q_rand)
                if test_norm < current_min_norm
                    
                    % TODO may not need to reassign like this
                    q_near = HLP.nodes(i)
                    q_near_idx = i
                    current_min_norm = test_norm
                end
                
            end
        end
        
        % Generate a new configuration that is d_edge distance from q_near,
        % in the direction of q_rand
        % Add new config to nodes[]
        function q_new = generate_new_config(HLP, q_rand, q_near_idx)
            
            % get direction vector, then scale by distance
            % TODO test this
            disp('generating new config')
            q_near = HLP.nodes(1:HLP.dimension, q_near_idx)
            q_new = HLP.d_edge * (q_rand - q_near) + q_near
            
        end
        
        % Collision checking function - look through obstacles
        function check = collsion_check(HLP, q_new, obstacles)
            
            disp('Implement collsion check suitable for subclass here');
            check = 0;
            % complete this
            % Method will change depending on dimension
            % (SurfaceIntersection vs polyxpoly)
            % TODO create 2 and 3D subclasses
        end
        
        % add new node if valid
        function add_node(HLP, q_new, q_near_idx)
            % add node
            HLP.nodes = [HLP.nodes; q_new];
           
            % add parent
            
            HLP.node_parent_idx = [HLP.node_parent_idx; q_near_idx];
        end
        
        
        
    end
end

