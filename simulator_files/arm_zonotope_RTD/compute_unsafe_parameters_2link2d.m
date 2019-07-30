function [links, k_lim, k_unsafe_A, k_unsafe_b] = compute_unsafe_parameters_2link2d(varargin)
%COMPUTE_UNSAFE_PARAMETERS_2LINK2D generates constraints on trajectory
%   parameters given a set of obstacles.
%   COMPUTE_UNSAFE_PARAMETERS_2LINK2D(theta_0, theta_dot_0, obstacles) uses
%   the initial configuration (vector theta_0) and initial velocities
%   (theta_dot_0) to construct an appropriate FRS of the arm over a range
%   of trajectory parameters. Then, it intersects the FRS with the
%   obstacles (cell of zonotopes) to generate constraints describing unsafe
%   trajectory paramters.

switch nargin
    case 3
        theta_0 = varargin{1};
        theta_dot_0 = varargin{2};
        obstacles = varargin{3};
        options = struct();
        options.t_plan = 0.5;
        options.t_stop = 1;
        options.position_dimensions = [1;2]; % x and y dimensions
        options.param_dimensions = [3;4]; % theta_dot_0 and theta_dot_pk dimensions
    case 4
        theta_0 = varargin{1};
        theta_dot_0 = varargin{2};
        obstacles = varargin{3};
        options = varargin{4};
    otherwise
        error('Specify theta_0, theta_dot_0, and obstacles');
end

links = stack_FRS_2link2d(theta_0, theta_dot_0); % this also slices at the correct initial velocity
nLinks = length(links);
nObs = length(obstacles);

k_lim = cell(nLinks, 1);
k_unsafe_A = cell(nLinks, 1);
k_unsafe_b = cell(nLinks, 1);
for i = 1:nLinks
% for i = 2
    k_lim{i} = get_parameter_limits(links{i});
    k_unsafe_A{i} = cell(nObs, 1);
    k_unsafe_b{i} = cell(nObs, 1);
    for j = 1:nObs
        [k_unsafe_A{i}{j}, k_unsafe_b{i}{j}] = get_parameter_constraints(links{i}, k_lim{i}, obstacles{j});
    end
end


end

