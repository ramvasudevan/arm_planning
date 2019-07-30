% function [ ] = compute_arm_vol_FRS( )
% Generate volumetric FRS for arm

generate_arm_vol_dynamics_absolute();
l1 = 0.5;
l2 = 0.5;
% l3 = 0.5;
dt = 0.002;
dim = 7;

% Compute FRS
%set options
options.tStart = 0;
options.tFinal = l1;
options.x0 = zeros(dim, 1);
options.R0 = zonotope([options.x0, diag([0, 0, 1, 1, 1, 1, 0])]); % cos and sin in -1 to 1 box
options.timeStep = dt;
options.taylorTerms=5; %number of taylor terms for reachable sets
options.zonotopeOrder= 20; %zonotope order... increase this for more complicated systems.
options.maxError = 1000*ones(dim, 1);
options.verbose = 1;

options.uTrans = 0;
options.U = zonotope([0, 0]);

options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';

%specify 1st link continuous dynamics-----------------------------------------------
sys_l1 = nonlinearSys(dim, 1, @arm_vol_dyn_abs_l1, options);
%compute reachable set-----------------------------------------------------
tic
Rcont_l1 = reach(sys_l1, options);
tComp = toc;
disp(['computation time of reachable set: ', num2str(tComp)]);

% specify 2nd link continuous dynamics
options.tFinal = l2;
options.R0 = Rcont_l1{end}{1};
sys_l2 = nonlinearSys(dim, 1, @arm_vol_dyn_abs_l2, options);
% compute 2nd reachable set -----
tic
Rcont_l2 = reach(sys_l2, options);
tComp = toc;
disp(['computation time of reachable set: ', num2str(tComp)]);

Rcont = [Rcont_l1; Rcont_l2];

theta_a = [pi/3, pi/3+0.2];
theta_b = [pi/8, pi/3];

% try slicing a couple
for i = 1:length(Rcont)
    
%     Rcont_a{i}{1} = zonotope_slice(Rcont{i}{1}, [3;4;5;6;7;8], [cos(theta_a(1)); cos(theta_a(1) + theta_a(2)); cos(theta_a(1) + theta_a(2) + theta_a(3)); sin(theta_a(1)); sin(theta_a(1) + theta_a(2));  sin(theta_a(1) + theta_a(2) + theta_a(3))]);
%     Rcont_b{i}{1} = zonotope_slice(Rcont{i}{1}, [3;4;5;6;7;8], [cos(theta_b(1)); cos(theta_b(1) + theta_b(2)); cos(theta_b(1) + theta_b(2) + theta_b(3)); sin(theta_b(1)); sin(theta_b(1) + theta_b(2));  sin(theta_b(1) + theta_b(2) + theta_b(3));]);

    Rcont_a{i}{1} = zonotope_slice(Rcont{i}{1}, [3;4;5;6], [cos(theta_a(1)); sin(theta_a(1)); cos(theta_a(2)); sin(theta_a(2))]);
    Rcont_b{i}{1} = zonotope_slice(Rcont{i}{1}, [3;4;5;6], [cos(theta_b(1)); sin(theta_b(1)); cos(theta_b(2)); sin(theta_b(2))]);

end

figure(1); clf; hold on;
for i = 1:length(Rcont)
    pa = plotFilled(Rcont_a{i}{1}, [1, 2], 'b');
    pb = plotFilled(Rcont_b{i}{1}, [1, 2], 'r');
    pa.FaceAlpha = 0.1;
    pa.EdgeAlpha = 0;
    pb.FaceAlpha = 0.1;
    pb.EdgeAlpha = 0;
end
axis equal; axis square;

% save the arm FRS
save('arm_FRS_twoLink', 'Rcont', 'options','l1', 'l2');

% end

