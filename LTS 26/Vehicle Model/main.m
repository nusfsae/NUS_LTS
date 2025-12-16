%% Run this program to initiate simulation


% Contact:
% Lap Time Simulation department, NUS Formula SAE R26 Team
% Dai Baizhou Patrick
% patrick.dai@yahoo.com



%% load files
addpath('C:\Users\PC5\Documents\casadi-3.6.7-windows64-matlab2018b')
addpath(genpath(cd))

%% enter settings:

% Chassis Settings
car = 199.6;                         % vehicle mass (kg)
driver = 65;                         % driver mass (kg)
track = 1.21;                        % track width (m)
cg_f = 0.5095;                       % mass bias to front (-)
wheelbase = 1.531;                   % wheelbase (m)
cg_h = 0.256;                        % CG height (m)
% Suspension Settings
del_max = 0.565;                     % maximum steering angle (rad)
R = 0.2032;                          % wheel radius (m)
P = 9;                               % tire pressue (psi)
IA = 0;                              % inclination angle (rad)
brakebias = 0.67;                    % brake bias
% Ackerman Settings
AckSource = readmatrix("ackerman.xlsx");   
[~, ~, p_inner, p_outer] = acker([], AckSource);
save('ackerman_coeffs.mat', 'p_inner', 'p_outer');
% Tyre Settings
para = H1675;                        % tire selection
% Aerodynamics Settings
den = 1.196;                         % air density (kgm^-3)
farea = 1.157757;                    % frontel area (m^2)
CLc = 3.828;                         % CL cornering
CDc = 1.403;                         % CD cornering
ab_c = 0.528;                        % aero balance cornering (front)
CLs = 4.034;                         % CL straight line
CDs = 1.543;                         % CD straight line
ab_s = 0.549;                        % aero balance straight line (front)
% Powertrain Settings 
max_rpm = 5500;                      % maximum wheel speed (rpm)
FDR = 3.36;                          % final drive ratio (-)
Ipeak = 1  ;                         % power percentage (-)
PMaxLimit = 80;                      % power limit (KW)
% Race Track
endurance = 'Endurance.mat';
skidpad = 'Skidpad.mat';
acceleration = 'Acceleration.mat';
% Rolling/Standing start
static = true;

%% run performance envelope
GGV;

%% initialize/reset simulation results
load(endurance);
num = length(C2);
sim = struct();

%% dynamic simulation
dynamics;

%% performance data plots
plotter;

%% summary
fprintf('*******Simulation Summary*******\n');
fprintf("Simulated lap time: %.3f (seconds)\n", t(end));
fprintf("Maximum Lateral Acceleration: %.3f (G)\n",max(abs(sim.ay))/9.81);
fprintf("Maximum Acceleration: %.3f (G)\n", max(sim.ax)/9.81);
fprintf("Maximum Braking: %.3f (G)\n", min(sim.ax)/9.81);
fprintf("Number of optimization failure: %d \n", failcount);
fprintf("Percentage of optimization failure: %.3f (%%)\n", failcount*100/(Vnum*Gnum));

