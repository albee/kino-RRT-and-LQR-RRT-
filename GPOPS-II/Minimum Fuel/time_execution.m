% time_execution.m

close all
clearvars

% Define number of computation time samples to take [-]
N = 1;

% Define trajectory time [s]
T = 60;

% Define initial state
x0 = [10;...    % Initial x position [m]
      10;...    % Initial y position [m]
      0;...     % Initial x velocity [m/s]
      0;...     % Initial y velocity [m/s]
      0;...     % Initial heading [rad]
      0];       % Initial angular velocity [rad/s]
  
% Define goal state
xF = [0;...     % Goal x position [m]
      0;...     % Goal y position [m]
      0;...     % Goal x velocity [m/s]
      0;...     % Goal y velocity [m/s]
      0;...     % Goal heading [rad]
      0];       % Goal angular velocity [rad/s]

% Define operational state of each thruster (1 = nominal, 0 = failed)
state = [0;...
         0;...
         1;...
         1];
  
% Load parameters
params = load_params();
params.T     = T;
params.x0    = x0;
params.xF    = xF;
params.state = state;

% Set up GPOPS-II
setup = setup_gpops(params);

% Initialize vector to store output
comp_time = zeros(N,1);

% Time solving
for n = 1:N
    output = gpops2(setup);
    comp_time(n) = output.totaltime;
end

% Plot
figure
hold on
ax = gca;
ax.FontSize = 20;
ax.LineWidth = 1.5;
ax.TickLabelInterpreter = 'latex';
fig = gcf;
fig.PaperPositionMode = 'auto';
fig_pos = fig.PaperPosition;
fig.PaperSize = [fig_pos(3) fig_pos(4)];
xlabel('Computation Time [s]','FontSize',24,'Interpreter','latex')
ylabel('PDF','FontSize',24,'Interpreter','latex')
histogram(comp_time,'Normalization','pdf','BinWidth',0.025,'FaceColor',[0.75 0.75 0.75],'FaceAlpha',1,'LineWidth',1)

% Save data
save twofail_time comp_time