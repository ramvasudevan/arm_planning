classdef arm_world_static < world
    properties
        include_base_obstacle = true ;
        obstacle_size_range = [0.05 0.15] ; % [min, max] side length
        obstacle_creation_timeout = 1 ; % seconds per obstacle
        start_creation_timeout = 3 ; % seconds allowed to create start
        
        % goal plotting
        goal_plot_patch_data
        goal_plot_face_color = [0 1 0] ;
        goal_plot_face_alpha = 0.1 ;
        goal_plot_edge_color = [0 1 0] ;
        goal_plot_edge_alpha = 0.3 ;
        goal_plot_edge_style = '--' ;
    end
    
    methods
        %% constructor
        function W = arm_world_static(varargin)
            % W = arm_world_static('property1',value1,'property2',value2,...)
            W@world('start',[],'goal',[],'N_obstacles',0,'verbose',10,...
                varargin{:}) ;
            
            W.plot_data.obstacles = [] ;
            W.plot_data.goal = [] ;
        end
        
        %% setup
        function setup(W,I)
            W.vdisp('Running arm world setup',1)
            
            % set world bounds based on agent limits
            W.bounds = I.reach_limits ;
            
            % set any joint limits that are +Inf to pi and -Inf to -pi
            joint_limits = I.joint_limits ;
            joint_limit_infs = isinf(joint_limits) ;
            joint_limits(1,joint_limit_infs(1,:)) = -pi ;
            joint_limits(2,joint_limit_infs(2,:)) = +pi ;
            
            % create base obstacle (this is like a mounting point for
            % the arm)
            if W.include_base_obstacle
                W.vdisp('Making base obstacle',3) ;
                W.obstacles = {W.create_base_obstacle()} ;
            end
            
            % create random start configuration that is not in collision
            % with the base obstacle
            if isempty(W.start)
                W.vdisp('Making start configuration',5)
                
                start_config = rand_range(joint_limits(1,:),joint_limits(2,:))' ;
                
                if W.include_base_obstacle
                    W.vdisp('Getting valid start configuration',6)
                    
                    start_valid = false ;
                    start_tic = tic ;
                    t_cur = toc(start_tic) ;
                    O = W.obstacles{1} ;
                    
                    while ~start_valid && t_cur < W.start_creation_timeout
                        V_arm = I.get_collision_check_volume(start_config) ;
                        
                        switch W.dimension
                            case 2
                                V_obs = O.collision_check_patch_data.vertices ;
                                [x_int,~] = polyxpoly(V_arm(1,:)',V_arm(2,:)',...
                                    V_obs(:,1),V_obs(:,2)) ;
                                if isempty(x_int)
                                    start_valid = true ;
                                end
                                
                            case 3
                                O_str.faces = O.collision_check_patch_data.faces ;
                                O_str.vertices = O.collision_check_patch_data.vertices ;
                                check = SurfaceIntersection(O_str,V_arm) ;
                                start_valid = ~any(check(:)) ;
                        end
                        
                        start_config = rand_range(joint_limits(1,:),joint_limits(2,:))' ;
                    end
                end
                
                if start_valid
                    W.start = start_config ;
                else
                    error('Unable to find valid starting configurations!')
                end
            end
            
            % create random goal configuration
            if isempty(W.goal)
                W.vdisp('Making goal configuration',5)
                W.goal = rand_range(joint_limits(1,:),joint_limits(2,:))' ;
                W.goal_plot_patch_data = I.get_collision_check_volume(W.goal) ;
            end
            
            % get arm volume at initial configuration
            V_arm = I.get_collision_check_volume(W.start) ;
            
            % create obstacles
            W.vdisp('Creating obstacles!',2) ;
            W.obstacles = [W.obstacles, cell(1,W.N_obstacles)] ;
            
            for idx = 1:W.N_obstacles
                W.vdisp(['Making obstacle ',num2str(idx)],3) ;
                
                % try creating a box obstacle that doesn't intersect
                % the initial configuration of the arm
                obs_ok_flag = false ;
                start_tic = tic ;
                t_cur = toc(start_tic) ;
                while ~obs_ok_flag && t_cur < W.obstacle_creation_timeout
                    % create center
                    B = W.bounds ;
                    
                    center = [rand_range(B(1),B(2)) ; rand_range(B(3),B(4))] ;
                    
                    if W.dimension == 3
                        center = [center ; rand_range(B(5),B(6))] ;
                    end
                    
                    % create side lengths
                    side_lengths = rand_range(W.obstacle_size_range(1),...
                        W.obstacle_size_range(2),[],[],1,W.dimension) ;
                    
                    % create candidate obstacle
                    O = box_obstacle_zonotope('center',center(:),...
                        'side_lengths',side_lengths) ;
                    
                    W.vdisp('Collision checking new obstacle',5)
                    switch W.dimension
                        case 2
                            V_obs = O.collision_check_patch_data.vertices ;
                            [x_int,~] = polyxpoly(V_arm(1,:)',V_arm(2,:)',...
                                V_obs(:,1),V_obs(:,2)) ;
                            if isempty(x_int)
                                obs_ok_flag = true ;
                            end
                            
                        case 3
                            O_str.faces = O.collision_check_patch_data.faces ;
                            O_str.vertices = O.collision_check_patch_data.vertices ;
                            check = SurfaceIntersection(O_str,V_arm) ;
                            obs_ok_flag = ~any(check(:)) ;
                    end
                    t_cur = toc(start_tic) ;
                end
                
                if obs_ok_flag
                    W.vdisp('Obstacle created!',3)
                    W.obstacles{idx+1} = O ;
                else
                    W.vdisp('Obstacle creation failed',3)
                end
            end
            
            W.N_obstacles = length(W.obstacles) ;
            
            W.vdisp('Arm world setup complete',2)
        end
        
        %% make obstacles
        function O = create_base_obstacle(W)
            switch W.dimension
                case 2
                    side_lengths = [W.bounds(2) - W.bounds(1), 0.05] ;
                    center = [0 ; -side_lengths(2)/2] ;
                case 3
                    side_lengths = [W.bounds(2) - W.bounds(1), ...
                        W.bounds(4) - W.bounds(3),...
                        0.05] ;
                    center = [0 ; 0 ; -side_lengths(3)/2] ;
            end
            
            O = box_obstacle_zonotope('center',center,...
                'side_lengths',side_lengths,...
                'plot_face_color',[0.1 0.1 0.5],...
                'plot_edge_color',[0 0 1]) ;
        end
        
        %% plotting
        function plot(W)
            hold_check = ~ishold ;
            if hold_check
                hold on ;
            end
            
            % plot obstacles (each obstacle takes care of its own plotting)
            W.vdisp('Plotting obstacles',6)
            for idx = 1:W.N_obstacles
                plot(W.obstacles{idx}) ;
            end
            
            % plot goal config
            W.plot_goal()
            
            if hold_check
                hold off
            end
        end
        
        function plot_goal(W)
            W.vdisp('Plotting goal',6)
            
            hold_check = ~ishold ;
            if hold_check
                hold on ;
            end
            
            G = W.goal_plot_patch_data ;
            
            if ~check_if_plot_is_available(W,'goal')
                switch W.dimension
                    case 2
                        data = plot(G(1,:),G(2,:),'Color',W.goal_plot_edge_color,...
                            'LineStyle',W.goal_plot_edge_style) ;
                    case 3
                        data = patch(G,...
                            'LineStyle',W.goal_plot_edge_style,...
                            'FaceColor',W.goal_plot_face_color,...
                            'FaceAlpha',W.goal_plot_face_alpha,...
                            'EdgeColor',W.goal_plot_edge_color,...
                            'EdgeAlpha',W.goal_plot_edge_alpha) ;
                end
                
                W.plot_data.goal = data ;
            end
            
            if hold_check
                hold off
            end
        end
    end
end