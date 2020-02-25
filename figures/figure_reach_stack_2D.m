% patrick 20200126
% making a figure that shows the reachability, stacking, and slicing ideas
% all in one

clear; clc;
figure(1); clf; hold on;
figure(2); clf; hold on;
figure(3); clf; hold on;
figure(4); clf; hold on;
figure(5); clf; hold on;

blues = 1/256*[222,235,247;
158,202,225;
49,130,189];

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
sliceK = [deltaK/2; -1*deltaK/2];

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
q_plan = (q_0 + v_0*t_plan + 1/2*sliceK*t_plan^2);
q_dot_plan = v_0 + sliceK*t_plan;
q_final = q_plan + 0.5*q_dot_plan*(t_total - t_plan);


% PLOTTING
% use 5 different figures:
save_flag = 1;
FaceAlphaLight = 0.1;
EdgeAlphaLight = 0.08;
FaceAlpha = 0.5;
EdgeAlpha = 0.4;

% plot_slice_flag = true;
% plot_cos_sin_flag = true;
% plot_link_flag = true;

j_range = [1:length(Rcont{1})];
% j_range = [length(Rcont{1})];

color = blues(1, :);
slicecolor = blues(2, :);
linkcolor = blues(3, :);
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
        
%         if plot_slice_flag
%             p2 = plotFilled(zonotope_slice(Rcont{i}{j}{1}, [3; 4], [sliceK(i); v_0(i)]), [1, 2], slicecolor);
%             p2.FaceAlpha = FaceAlpha;
%             p2.EdgeAlpha = EdgeAlpha;
%         end
        
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
%     if plot_slice_flag
        p2 = plot_slice(link_FRS{i}{100}, sliceK(link_joints{i}), slicecolor, 2);
        p2.FaceAlpha = FaceAlpha;
        p2.EdgeAlpha = EdgeAlpha;
%     end
    
%     if plot_link_flag
        p3 = plot([0, L(1)*cos(q_final(1)), L(1)*cos(q_final(1)) + L(2)*cos(q_final(1) + q_final(2))], ...
            [0, L(1)*sin(q_final(1)), L(1)*sin(q_final(1)) + L(2)*sin(q_final(1) + q_final(2))], ...
            'Color', linkcolor, 'LineWidth', 5, 'Marker', '.', 'MarkerSize', 20);
%     end
end
axis equal;

% formatting
% subplot(2,2,3);
figure(1);
axis equal;
set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
xlabel('$\cos(q_1)$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$\sin(q_1)$', 'Interpreter', 'latex', 'FontSize', 24);
x_limits = [0, 1];
x_ = linspace(x_limits(1), x_limits(2), 100);
y_ = sqrt(1 - x_.^2);
plot(x_, y_, 'k--', 'LineWidth', 1);

figure(2);
axis equal;
set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
xlabel('$\cos(q_2)$', 'Interpreter', 'latex', 'FontSize', 24);
ylabel('$\sin(q_2)$', 'Interpreter', 'latex', 'FontSize', 24);
x_limits = [0, 1];
x_ = linspace(x_limits(1), x_limits(2), 100);
y_ = sqrt(1 - x_.^2);
plot(x_, y_, 'k--', 'LineWidth', 1);

% if plot_cos_sin_flag
%     ax = gca;
%     x_limits = ax.XLim;
%     delta_x = sum(abs([1 0]*Rcont{1}{100}{1}.Z(1:2, 2:end)));
%     x_ = linspace(Rcont{1}{100}{1}.Z(1, 1)-delta_x, Rcont{1}{100}{1}.Z(1, 1)+delta_x, 100);
    x_limits = [0.241, 0.5159];
    x_ = linspace(x_limits(1), x_limits(2), 100);
    y_ = sqrt(1 - x_.^2);
    plot(x_, y_, 'k--', 'LineWidth', 1);
% end

figure(3);
axis equal;
% set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
xlabel('$\cos(q_1)$', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('$\sin(q_1)$', 'Interpreter', 'latex', 'FontSize', 15);
x_limits = [0.241, 0.5159];
x_ = linspace(x_limits(1), x_limits(2), 100);
y_ = sqrt(1 - x_.^2);
plot(x_, y_, 'k--', 'LineWidth', 1);
plot(cos(q_final(1)), sin(q_final(1)), 'Marker', '.', 'MarkerSize', 30, 'Color', linkcolor);
% end

figure(4);
axis equal;
% set(gca, 'XLim', [0, 1], 'YLim', [0, 1]);
xlabel('$\cos(q_2)$', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('$\sin(q_2)$', 'Interpreter', 'latex', 'FontSize', 15);

% if plot_cos_sin_flag
%     ax = gca;
%     x_limits = ax.XLim;
%     delta_x = sum(abs([1 0]*Rcont{2}{100}{1}.Z(1:2, 2:end)));
%     x_ = linspace(Rcont{2}{100}{1}.Z(1, 1)-delta_x, Rcont{2}{100}{1}.Z(1, 1)+delta_x, 100);
    x_limits = [0.7429, 0.9016];
    x_ = linspace(x_limits(1), x_limits(2), 100);
    y_ = sqrt(1 - x_.^2);
    plot(x_, y_, 'k--', 'LineWidth', 1);
    plot(cos(q_final(2)), sin(q_final(2)), 'Marker', '.', 'MarkerSize', 30, 'Color', linkcolor);
% end


% subplot(2,2,[2, 4]);
figure(5);
axis equal;
xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 15);
ylabel('$y$', 'Interpreter', 'latex', 'FontSize', 15);

if save_flag
%     if plot_slice_flag
%     else
%         filename = sprintf('figure_reach_stack_2D');
%     end

filename = sprintf('figure_reach_stack_2D_slice');
    for i = 1:5
        print([filename '_' num2str(i)], '-dpng', '-r600');
        print([filename '_' num2str(i)], '-depsc', '-r600');
    end
end