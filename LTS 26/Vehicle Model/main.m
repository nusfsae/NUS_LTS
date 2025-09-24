%% Run this program to initiate simulation


% Contact:
% Lap Time Simulation department, NUS Formula SAE R26 Team
% Dai Baizhou Patrick
% patrick.dai@yahoo.com

clear

%% load files
addpath('C:\Users\Patri\casadi-3.6.7-windows64-matlab2018b')
addpath(genpath(cd))

%% enter settings:

% Chassis Settings
mass = 262;                          % vehicle mass (kg)
track = 1.21;                        % track width (m)
cg_f = 0.5095;                       % mass bias to front (-)
wheelbase = 1.531;                   % wheelbase (m)
cg_h = 0.256;                        % CG height (m)
% Suspension Settings
del_max = 0.565;                     % maximum steering angle (rad)
R = 0.2032;                          % wheel radius (m)
P = 9;                               % tire pressue (psi)
IA = 0;                              % inclination angle (rad)
% Tyre Settings
para = H1675;                        % tire selection
% Aerodynamics Settings
den = 1.196;                         % air density (kgm^-3)
farea = 1.157757;                    % frontel area (m^2)
CLc = 3.782684;                      % CL cornering
CDc = 1.410518;                      % CD cornering
CLs = 4.116061;                      % CL straight line
CDs = 1.54709;                       % CD straight line
ab = 0.5310665;                      % aero balance (front)
% Powertrain Settings
max_rpm = 5500;                      % maximum wheel speed (rpm)
FDR = 3.00;                          % final drive ratio (-)
Ipeak = 0.5;                         % power percentage (-)
PMaxLimit = 80;                      % power limit (KW)
% Race Track
load('25 Endurance.mat');

figure

%% initialize/reset simulation results
num = length(C2);
sim = struct();
sim.ay = zeros(num,1);
sim.ax = zeros(num,1);
sim.speed = zeros(num,1);
sim.torque = zeros(num,1);
sim.accel = zeros(num,1);
sim.brake = zeros(num,1);
sim.throttle = zeros(num,1);

%% run performance envelope
GGV;

%% dynamic simulation
dynamics;

%% performance data plots

% calculate yaw rate
sim.yaw = sim.ay./sim.speed;
% calculate lap time
dt = 1./sim.speed;
t = cumtrapz(dist,dt);
laptime = t(end);

figure
tiledlayout(5,1);
% plot speed profile
nexttile;
plot(dist,sim.speed*3.6);ylabel('speed (km/h)');ylim([0 140]);
title('Vehicle Speed')
% plot ay
nexttile
plot(dist,sim.ay/9.81);ylabel('ay (G)');ylim([-3 3]);
title('Lateral Acceleration')
% plot ax
nexttile
plot(dist,sim.ax/9.81);ylabel('ax (G)');ylim([-3 3]);
title('Longitudinal Acceleration')
% plot yaw rate
nexttile
plot(dist,rad2deg(sim.yaw));ylabel('dpsi (deg/s)');ylim([-180 180]);
title('Yaw Rate')
% plot time
nexttile
plot(dist,t);ylabel('time (s)');ylim([0 t(end)]);
title('Time')
fprintf("Simulated lap time is %.3f seconds\n ", t(end));
xlabel('distance(m)')

% plot color track map speed data
figure
scatter(pos.x,pos.y,10,sim.speed*3.6,'filled','o');
colormap(jet);
cb = colorbar;
ylabel(cb, 'Speed (km/h)');
title('Race Track with Speed Visualization', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
grid on;
axis equal;
