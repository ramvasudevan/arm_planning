% patrick 20200126
% making a figure that shows the reachability, stacking, and slicing ideas
% all in one

clear; clc;
figure(1); clf; hold on;
figure(2); clf; hold on;
figure(3); clf; hold on;
figure(4); clf; hold on;
figure(5); clf; hold on;
figure(6); clf; hold on;

blues = 1/256*[222,235,247;
158,202,225;
49,130,189];

% create obstacle
obs = box_obstacle_zonotope('center', [-0.019; 0.5; 0], 'side_lengths', 0.15*ones(3, 1));
figure(5); plot(obs);

% some user parameters:
q_0(1, 1) = 0;
q_0(2, 1) = 0;
v_0(1, 1) = pi/2;
v_0(2, 1) = pi/4;
deltaV = pi/200;
deltaK = pi/6;
L = [0.33; 0.33];
link_zonotopes = {zonotope([L(1)/2, L(1)/2; 0, 0; 0, 0]); zonotope([L(2)/2, L(2)/2; 0, 0; 0, 0])};
link_EE_zonotopes = {zonotope([L(1); 0; 0])};
rot_axes = [3; 3];
link_joints = {[1], [1, 2]};
% sliceK = [0.202; 0.303];
sliceK = [-0.4;0.3];
% sliceK = [deltaK/2; -1*deltaK/2];
% goalK = [0.3583; 0.3583];
goalK = [deltaK-0.1; deltaK-0.1];


% first, compute some reach sets:
dim = 5;
t_plan = 0.5;
t_total = 1;
dt = 0.01;

for i = 1:2
    options.tStart = 0;
    options.tFinal = t_plan;
    options.x0 = [1; 0; 0; v_0(i); 0];
    options.R0 = zonotope([options.x0, diag([0, 0, deltaK, deltaV, 0])]);
    options.timeStep = dt;
    options.taylorTerms=5; %number of taylor terms for reachable sets
    options.zonotopeOrder= 4; %zonotope order... increase this for more complicated systems.
    options.maxError = 1000*ones(dim, 1);
    options.verbose = 1;
    options.uTrans = 0;
    options.U = zonotope([0, 0]);
    options.advancedLinErrorComp = 0;
    options.tensorOrder = 1;
    options.reductionInterval = inf;
    options.reductionTechnique = 'girard';
    sys = nonlinearSys(dim, 1, @trig_dyn_toPeak, options);
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toPeak = reach(sys, options);
    tComp = toc;

    % then use the computed FRS as the initial FRS for the braking dynamics.
    options.R0 = Rcont_toPeak{end}{1};
    % however, slice the zonotope right at the t_plan time:
    options.R0 = zonotope_slice(options.R0, [dim], t_plan);
    options.tStart = t_plan;
    options.tFinal = t_total;
    %specify 1st link to stop dynamics------------------------------------------
    sys = nonlinearSys(dim, 1, @trig_dyn_toStop, options);
    %compute reachable set-----------------------------------------------------
    tic
    Rcont_toStop = reach(sys, options);
    tComp = toc;

    % concatenate full FRS
    Rcont{i} = [Rcont_toPeak; Rcont_toStop];
end

% create rotatotopes
link_rotatotopes = {};
link_EE_rotatotopes = {};
for i = 1:length(Rcont)
    A = [cos(q_0(i)), -sin(q_0(i)), 0, 0, 0; sin(q_0(i)), cos(q_0(i)), 0, 0, 0;...
        0, 0, 1, 0, 0; 0, 0, 0, 1, 0; 0, 0, 0, 0, 1];
    for j = 1:length(Rcont{i})
        trig_FRS{j}{i} = A*zonotope_slice(Rcont{i}{j}{1}, 4, v_0(i));
    end
end

% link rotatotopes
for i = 1:length(Rcont)
    for j = 1:length(Rcont{i})
        link_rotatotopes{i}{j} = rotatotope(rot_axes(link_joints{i}), trig_FRS{j}(link_joints{i}), link_zonotopes{i});
    end
end

% link EE rotatotopes
for i = 1:length(Rcont) - 1
    for j = 1:length(Rcont{i})
        link_EE_rotatotopes{i}{j} = rotatotope(rot_axes(link_joints{i}), trig_FRS{j}(link_joints{i}), link_EE_zonotopes{i});
    end
end

% stack
link_FRS = link_rotatotopes;
for i = 1:length(Rcont)
    for j = 1:length(Rcont{i})
        % stack on prev. links
        for k = 1:i-1
            link_FRS{i}{j} = link_FRS{i}{j}.stack(link_EE_rotatotopes{k}{j});
        end
    end
end

% compute actual final point
T = 0:dt:t_total ;
% T = linspace(0, t_total, 100);
T_plan = T(1:floor(length(T)/2)+1) ;
T_brake = T(floor(length(T)/2)+1:end) ;

t_to_stop = T_brake(end) - T_brake(1);

q_to_peak = q_0 + v_0.*T_plan + (1/2)*sliceK.*T_plan.^2;
q_dot_to_peak = v_0 + sliceK.*T_plan;

q_peak = q_to_peak(:, end);
q_dot_peak = q_dot_to_peak(:, end);

T_brake = T_brake - T_brake(1);
q_to_stop = q_peak + q_dot_peak.*T_brake + (1/2)*((0 - q_dot_peak)./t_to_stop).*T_brake.^2;
q_dot_to_stop = q_dot_peak + ((0 - q_dot_peak)./t_to_stop).*T_brake;

% remove overlap
q_to_stop(:, 1) = [];
q_dot_to_stop(:, 1) = [];
q_traj = [q_to_peak, q_to_stop];

q_plan_goal = (q_0 + v_0*t_plan + 1/2*goalK*t_plan^2);
q_dot_plan_goal = v_0 + goalK*t_plan;
q_final_goal = q_plan_goal + 0.5*q_dot_plan_goal*(t_total - t_plan);

% compute set of unsafe K by discretizing, taking contours
% disp('generating constraint values!')
% FRS_options = struct();
% FRS_options.maxcombs = 300;
% FRS_options.combs = generate_combinations_upto(FRS_options.maxcombs);
% K_disc = linspace(-deltaK, deltaK, 20);
% [Xk, Yk] = meshgrid(K_disc, K_disc);
% Zk = [];
% for i = 1:length(K_disc)
%     for j = 1:length(K_disc)
%         k_test = [Xk(i, j); Yk(i, j)];
%         %ugh... manually generate polytope normals and evaluate sliced
%         %constraints
%         A = {};
%         h = [];
%         for p = 1:length(link_FRS)
% %             for q = 1:length(link_FRS{p})
%             for q = length(link_FRS{p})
%                 A{p}{q} = generate_polytope_normals(link_FRS{p}{q}, obs.zono.Z, FRS_options);
%             end
%         end
%         for p = 1:length(link_FRS)
%             idx = find(~cellfun('isempty', A{p}));
% %             idx = length(link_FRS{p});
%             for q = 1:length(idx)
%                 h(end+1, 1) = evaluate_sliced_constraints(link_FRS{p}{idx(q)}, k_test, obs.zono.Z, A{p}{idx(q)});
%             end 
%         end
%         if ~isempty(h)
%             Zk(i, j) = max(h);
%         else
%             Zk(i, j) = -inf;
%         end
%     end 
% end
% disp('finished constraint testing')
% constraint_eval.Xk = Xk;
% constraint_eval.Yk = Yk;
% constraint_eval.Zk = Zk;
constraint_eval = load('figure_constraint_eval_2.mat');


% PLOTTING
% use 5 different figures:
save_flag = 0;
FaceAlphaLight = 0.1;
EdgeAlphaLight = 0.12;
FaceAlpha = 0.5;
EdgeAlpha = 0.8;

% plot_slice_flag = true;
% plot_cos_sin_flag = true;
% plot_link_flag = true;

j_range = [1:1:length(Rcont{1})];
j_slice_range = [1, 25, 50, 75, 100];
% j_slice_range = 100;
% j_range = [length(Rcont{1})];

color = blues(1, :);
slicecolor = blues(2, :);
slicehardcolor = 1/256*[238, 244, 250];
linkcolor = blues(3, :);
% goalcolor = 1/256*[218,165,32];
goalcolor = 1/256*[0,187,51];
% plot reach sets and slices
% figure(1);
for i = 1:length(Rcont)
    %     if i == 1
    %         subplot(2, 2, 3); hold on;
    %     else
    %         subplot(2, 2, 1); hold on;
    %     end
    figure(i); hold on;
    for j = j_range
        p = plotFilled(Rcont{i}{j}{1}, [1, 2], color);
        p.FaceAlpha = FaceAlphaLight;
        p.EdgeAlpha = EdgeAlphaLight;
    end
    for j = j_slice_range(end)
        p2 = plotFilled(zonotope_slice(Rcont{i}{j}{1}, [3; 4], [sliceK(i); v_0(i)]), [1, 2], slicecolor);
        p2.FaceAlpha = FaceAlpha;
        p2.EdgeAlpha = EdgeAlpha;
    end
    for j = j_slice_range
        plot(cos(q_traj(i, j)), sin(q_traj(i, j)), 'Marker', '.', 'MarkerSize', 30, 'LineWidth', 2, 'Color', linkcolor);
    end
end

for i = 1:length(Rcont)
    figure(i+2); hold on;
    p = plotFilled(Rcont{i}{100}{1}, [1, 2], color);
    p.FaceAlpha = FaceAlpha;
    p.EdgeAlpha = EdgeAlpha;
    p2 = plotFilled(zonotope_slice(Rcont{i}{100}{1}, [3; 4], [sliceK(i); v_0(i)]), [1, 2], slicecolor);
    p2.FaceAlpha = FaceAlpha;
    p2.EdgeAlpha = EdgeAlpha;
    p2.LineWidth = 1.25;
    plot(cos(q_traj(i, end)), sin(q_traj(i, end)), 'Marker', '.', 'MarkerSize', 30, 'LineWidth', 2, 'Color', linkcolor);
end

% plot rotatotopes and slices
% subplot(2, 2, [2, 4]); hold on;
figure(5); hold on;
% for i = 1:length(link_FRS)
for i = length(link_FRS):-1:1
    for j = j_range
        p = plot(link_FRS{i}{j}, color, 2);
        p.FaceAlpha = FaceAlphaLight;
        p.EdgeAlpha = EdgeAlphaLight;       
    end

end
for i = length(link_FRS):-1:1
    for j = j_slice_range(end)
        p2 = plot_slice(link_FRS{i}{j}, sliceK(link_joints{i}), slicehardcolor, 2);
        p2.FaceAlpha = 1;
        p2.EdgeAlpha = EdgeAlpha;
        p2.LineWidth = 1.75;
    end
end
for j = j_slice_range
    if j ~= 1
%         j = j+1;
    end
    p3 = plot([0, L(1)*cos(q_traj(1, j)), L(1)*cos(q_traj(1, j)) + L(2)*cos(q_traj(1, j) + q_traj(2, j))], ...
        [0, L(1)*sin(q_traj(1, j)), L(1)*sin(q_traj(1, j)) + L(2)*sin(q_traj(1, j) + q_traj(2, j))], ...
        'Color', linkcolor, 'LineWidth', 6, 'Marker', '.', 'MarkerSize', 1);
end
axis equal;

% plot set of unsafe coefficients:
figure(6);
redcolor = [1 0.65 0.65];
contourf(constraint_eval.Xk, constraint_eval.Yk, constraint_eval.Zk, [0, 0], 'LineWidth', 2);
colormap(redcolor);

% plot set of unsafe slices in last cos/sin plots
% for i = 1:length(Rcont)
%     figure(i+2); hold on;
%     for j = 1:size(constraint_eval.Xk, 1)
%         for k = 1:size(constraint_eval.Xk, 2)
%             if constraint_eval.Zk(j, k) > 0
%                 wook = [constraint_eval.Xk(j, k); constraint_eval.Yk(j, k)];
%                 p2 = plotFilled(zonotope_slice(Rcont{i}{100}{1}, [3; 4], [wook(i); v_0(i)]), [1, 2], redcolor);
%                 p2.FaceAlpha = 1;
%                 p2.EdgeAlpha = EdgeAlpha;
%             end
%         end
%     end
% end

% plot the "cost" function for the optimization
blehpt = [0.3; -0.1];
kcost = linspace(-deltaK, deltaK, 100);
[Xcost, Ycost] = meshgrid(kcost, kcost);
Zcost = zeros(size(Xcost));
for i = 1:size(Xcost, 1)
    for j = 1:size(Xcost, 2)
        wook = [Xcost(i, j); Ycost(i, j)];
        q_plan_temp = (q_0 + v_0*t_plan + 1/2*wook*t_plan^2);
        q_dot_plan_temp = v_0 + wook*t_plan;
        q_final_temp = q_plan_temp + 0.5*q_dot_plan_temp*(t_total - t_plan);
        Zcost(i,j) = (q_final_temp(1) - q_final_goal(1))^2 + (q_final_temp(2) - q_final_goal(2))^2;
%         Zcost(i, j) = (0.2*(wook(1) - blehpt(1))^2 + 0.2*(wook(2) - blehpt(2))^2)*((wook(1) - goalK(1))^2 + (wook(2) - goalK(2))^2);
    end
end
cost_levels = 0.1*[0.01, 0.04, 0.09, 0.16, 0.25, 0.36, 0.49, 0.64, 0.81, 1];
% cost_levels = [0.01, 0.09, 0.25, 0.49, 0.81];
% contour(Xcost, Ycost, Zcost, cost_levels, 'LineColor', 'k', 'LineWidth', 1);
plot(sliceK(1), sliceK(2), 'Color', linkcolor, 'Marker', '.', 'MarkerSize', 50, 'LineWidth', 2);
% plot(goalK(1), goalK(2), 'Color', goalcolor, 'Marker', 'p', 'MarkerSize', 16, 'LineWidth', 2, 'MarkerFaceColor', goalcolor);

% formatting
% subplot(2,2,3);
figure(1);
axis equal;
set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
set(gca, 'XTick', [0, 1], 'YTick', [0, 1]);
set(gca, 'FontSize', 15);
xlabel('$\cos(q_1)$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$\sin(q_1)$', 'Interpreter', 'latex', 'FontSize', 24);
x_limits = [0, 1];
x_ = linspace(x_limits(1), x_limits(2), 100);
y_ = sqrt(1 - x_.^2);
plot(x_, y_, 'k-', 'LineWidth', 0.2);

figure(2);
axis equal;
set(gca, 'FontSize', 15);
set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
set(gca, 'XTick', [0, 1], 'YTick', [0, 1]);
xlabel('$\cos(q_2)$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$\sin(q_2)$', 'Interpreter', 'latex', 'FontSize', 24);
x_limits = [0, 1];
x_ = linspace(x_limits(1), x_limits(2), 100);
y_ = sqrt(1 - x_.^2);
plot(x_, y_, 'k-', 'LineWidth', 0.2);

% if plot_cos_sin_flag
%     ax = gca;
%     x_limits = ax.XLim;
%     delta_x = sum(abs([1 0]*Rcont{1}{100}{1}.Z(1:2, 2:end)));
%     x_ = linspace(Rcont{1}{100}{1}.Z(1, 1)-delta_x, Rcont{1}{100}{1}.Z(1, 1)+delta_x, 100);
%     x_limits = [0.241, 0.5159];
%     x_ = linspace(x_limits(1), x_limits(2), 100);
%     y_ = sqrt(1 - x_.^2);
%     plot(x_, y_, 'k-', 'LineWidth', 0.2);
% end

figure(3);
axis equal;
% set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
set(gca, 'XTick', [], 'YTick', []);
xlabel('$\cos(q_1)$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$\sin(q_1)$', 'Interpreter', 'latex', 'FontSize', 24);
x_limits = [0.241, 0.5159];
x_ = linspace(x_limits(1), x_limits(2), 100);
y_ = sqrt(1 - x_.^2);
plot(x_, y_, 'k-', 'LineWidth', 0.2);
% end

figure(4);
axis equal;
set(gca, 'XTick', [], 'YTick', []);
% set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
xlabel('$\cos(q_2)$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$\sin(q_2)$', 'Interpreter', 'latex', 'FontSize', 24);

% if plot_cos_sin_flag
%     ax = gca;
%     x_limits = ax.XLim;
%     delta_x = sum(abs([1 0]*Rcont{2}{100}{1}.Z(1:2, 2:end)));
%     x_ = linspace(Rcont{2}{100}{1}.Z(1, 1)-delta_x, Rcont{2}{100}{1}.Z(1, 1)+delta_x, 100);
    x_limits = [0.7429, 0.9016];
    x_ = linspace(x_limits(1), x_limits(2), 100);
    y_ = sqrt(1 - x_.^2);
    plot(x_, y_, 'k-', 'LineWidth', 0.2);
% end


% subplot(2,2,[2, 4]);
figure(5);
axis equal;
set(gca, 'XTick', [], 'YTick', []);
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 24);
% p3 = plot([0, L(1)*cos(q_final_goal(1)), L(1)*cos(q_final_goal(1)) + L(2)*cos(q_final_goal(1) + q_final_goal(2))], ...
%     [0, L(1)*sin(q_final_goal(1)), L(1)*sin(q_final_goal(1)) + L(2)*sin(q_final_goal(1) + q_final_goal(2))], ...
%     'Color', goalcolor, 'LineWidth', 6, 'Marker', '.', 'MarkerSize', 1);

figure(6);
axis equal;
set(gca, 'XTick', [], 'YTick', []);
xlabel('$k_1$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$k_2$', 'Interpreter', 'latex', 'FontSize', 24);

if save_flag
%     if plot_slice_flag
%     else
%         filename = sprintf('figure_reach_stack_2D');
%     end

filename = sprintf('figure_reach_stack_2D_slice');
    for i = 1:6
        figure(i); set(gca, 'FontSize', 24);
        print([filename '_' num2str(i)], '-dpng', '-r600');
        fig = gcf;
        save_figure_to_pdf(fig, [filename '_' num2str(i)]);
%         print([filename '_' num2str(i)], '-depsc', '-r600');
    end
end