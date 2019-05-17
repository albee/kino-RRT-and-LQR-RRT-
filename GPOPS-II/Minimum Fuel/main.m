% main.m

close all
clearvars

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

% Solve
output = gpops2(setup);
solution = output.result.interpsolution;

% Unpack solution
t = solution.phase.time;

x  = solution.phase.state(:,1);
y  = solution.phase.state(:,2);
vx = solution.phase.state(:,3);
vy = solution.phase.state(:,4);
h  = solution.phase.state(:,5);
w  = solution.phase.state(:,5);

u1 = solution.phase.control(:,1);
u2 = solution.phase.control(:,2);
u3 = solution.phase.control(:,3);
u4 = solution.phase.control(:,4);

% Plot trajectory
% plot_trajectory(solution,params);

% Plot heading
figure
hold on
ax = gca;
ax.FontSize = 20;
ax.LineWidth = 1.5;
xlabel('Time [s]','FontSize',24)
ylabel('Heading [rad]','FontSize',24)
plot(t,h,'k-','LineWidth',2)
xlim([0 T])
grid on

% Plot control
figure
hold on
ax = gca;
ax.FontSize = 20;
ax.LineWidth = 1.5;
xlabel('Time [s]','FontSize',24)
ylabel('Control Thrust [N]','FontSize',24)
plot(t,u1,'k-','LineWidth',2)
plot(t,u2,'r-','LineWidth',2)
plot(t,u3,'b-','LineWidth',2)
plot(t,u4,'g-','LineWidth',2)
xlim([0 T])
grid on

% Save to CSV file
csvwrite('gpops2_trajectory.csv',[solution.phase.state solution.phase.control])

% Save to .mat file
traj  = [solution.phase.time solution.phase.state];
input = solution.phase.control;
save GPOPS_traj_twofail traj
save GPOPS_input_traj_twofail input