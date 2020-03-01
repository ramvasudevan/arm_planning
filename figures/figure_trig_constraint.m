%% description
% This script generates a figure showing how we can transform joint limit
% constraints into a linear constraint for a decision variable that is the
% sine and cosine of a joint angle, as opposed to the joint angle itself.
%
% Authors: Shreyas Kousik and Zehui Lu
% Created: 29 Feb 2020
% Updated: not yet
%
%% user parameters
% lower and upper bounds of joint
q_lo = -pi/4 ;
q_hi = 2*pi/3 ;

% whether or not to save output
save_pdf_flag = true ;

%% generate figure data
% make the unit circle
th = linspace(0,2*pi) ;
S = [cos(th) ; sin(th)] ;

% make the feasible region of the unit circle
th_feas = linspace(q_lo,q_hi) ;
S_feas = [cos(th_feas) ; sin(th_feas)] ;

% convert lower and upper bounds to sin/cos bounds
c_lo = cos(q_lo) ;
c_hi = cos(q_hi) ;
s_lo = sin(q_lo) ;
s_hi = sin(q_hi) ;

% generate patch for feasible region
CS_feas = [c_lo c_hi ; s_lo s_hi] ;
CS_center = mean(CS_feas,2) ;
dCS = CS_feas(:,2) - CS_feas(:,1) ;
dCS = dCS./vecnorm(dCS) ; % normalize me cap'n
nCS = cross([dCS(:);0],[0;0;1]); % get orthogonal vector
nCS = nCS(1:2) ;
vert_feas = [CS_feas(:,1) - 2*dCS, CS_feas(:,2) + 2*dCS] ;
vert_feas = [vert_feas, vert_feas + repmat(2*nCS,1,2)] ;

%% plotting
f = figure(1) ; clf ; hold on ; axis square ; grid on

% move axes to center of plot
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';
ax.FontSize = 15 ;

% set ticks
xticks(-1:0.5:1)
yticks(-1:0.5:1)
axis([-1.2,1.2,-1.2,1.2])

% label axes
xlabel('$c_i$','interpreter','latex','FontSize',25)
ylabel('$s_i$','interpreter','latex','FontSize',25)

% plot patch of feasible region
patch('faces',[1 2 4 3],'vertices',vert_feas','facecolor','g','facealpha',0.1)

% plot unit circle
plot_path(S,'k--','linewidth',1)

% plot feasible region
plot_path(S_feas,'g-','linewidth',2)

% plot endpoints
plot_path(CS_feas,'g.','markersize',15)

% plot linear constraint boundary
plot_path(vert_feas(:,1:2),'k-','linewidth',1)

% plot linear constraint direction vector
quiver(CS_center(1),CS_center(2),dCS(1),dCS(2),'LineWidth',3,...
    'MaxHeadSize',0.75,'Color','k','AutoScaleFactor',0.6) ;

% plot normal vector
quiver(CS_center(1),CS_center(2),nCS(1),nCS(2),'LineWidth',3,...
    'MaxHeadSize',0.75,'Color','g','AutoScaleFactor',0.6) ;

%% save output
if save_pdf_flag 
    save_figure_to_pdf(f,'joint_limit_constraint.pdf')
end