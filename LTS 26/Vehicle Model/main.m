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
FDR = 3.36;                          % final drive ratio (-)
Ipeak = 1.0;                         % power percentage (-)
PMaxLimit = 80;                      % power limit (KW)
% Race Track
load('25 Endurance.mat');
% Rolling/Standing start
static = false;

figure

%% initialize/reset simulation results
num = length(C2);
sim = struct();
%% run performance envelope
GGV;

%% dynamic simulation
dynamics;

%% performance data plots

% calculate yaw rate
sim.yaw = sim.ay./sim.speed;
% calculate lap time
dt = 1./max(sim.speed,0.001);
if sim.speed(1) == 0
    dt(1) = 0;
end
t = cumtrapz(dist,dt);
laptime = t(end);
% calculate steering angle
sim.delta = rad2deg(findDelta(abs(sim.ay),sim.speed));
neg = find(sim.ay<0);
sim.delta(neg) = -1*sim.delta(neg);
% slip ratio
sim.Sxfl = findSxfl(sim.ax,sim.speed);
sim.Sxfr = findSxfr(sim.ax,sim.speed);
sim.Sxrl = findSxrl(sim.ax,sim.speed);
sim.Sxrr = findSxrr(sim.ax,sim.speed);
% slip angle
sim.Safl = rad2deg(findSafl(abs(sim.ay),sim.speed));
sim.Safr = rad2deg(findSafr(abs(sim.ay),sim.speed));
sim.Sarl = rad2deg(findSarl(abs(sim.ay),sim.speed));
sim.Sarr = rad2deg(findSarr(abs(sim.ay),sim.speed));
sim.Safl(neg) = -1*sim.Safl(neg);
sim.Safr(neg) = -1*sim.Safr(neg);
sim.Sarl(neg) = -1*sim.Sarl(neg);
sim.Sarr(neg) = -1*sim.Sarr(neg);

% car statistics plots
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
xlabel('distance(m)')

% tire data plots
% slip ratio
figure
tiledlayout(4,1);
nexttile;
plot(dist,sim.Sxfl);ylabel('Slip Ratio');ylim([-0.1 0]);
title('FL')
nexttile;
plot(dist,sim.Sxfr);ylabel('Slip Ratio');ylim([-0.1 0]);
title('FR')
nexttile;
plot(dist,sim.Sxrl);ylabel('Slip Ratio');ylim([-0.1 0.1]);
title('RL')
nexttile;
plot(dist,sim.Sxrr);ylabel('Slip Ratio');ylim([-0.1 0.1]);
title('RR')
% slip angle
figure
tiledlayout(4,1);
nexttile;
plot(dist,sim.Safl);ylabel('Slip Angle');ylim([-10 10]);
title('FL')
nexttile;
plot(dist,sim.Safr);ylabel('Slip Angle');ylim([-10 10]);
title('FR')
nexttile;
plot(dist,sim.Sarl);ylabel('Slip Angle');ylim([-10 10]);
title('RL')
nexttile;
plot(dist,sim.Sarr);ylabel('Slip Angle');ylim([-10 10]);
title('RR')

% driver control plots
figure
% plot steering
plot(dist,sim.delta);ylabel('Steering Angle (deg)');ylim([-rad2deg(del_max) rad2deg(del_max)]);
title('Steering Angle')


%%

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

%% summary
fprintf('*******Simulation Summary*******\n');
fprintf("Simulated lap time: %.3f (seconds)\n", t(end));
fprintf("Maximum Lateral Acceleration: %.3f (G)\n",max(abs(sim.ay))/9.81);
fprintf("Maximum Acceleration: %.3f (G)\n", max(sim.ax)/9.81);
fprintf("Maximum Braking: %.3f (G)\n", min(sim.ax)/9.81);
fprintf("Number of optimization failure: %d \n", failcount);
fprintf("Percentage of optimization failure: %.3f (%%)\n", failcount*100/(Vnum*Gnum));

