% Main Script to Run Quasi Steady State LapSim

%%%%%%%%%%%%%%%%%%%%
% 11/08/2025
% Objectives:
% 1. Legible Code Flow
% 2. Clean workspace for debugging
% 3. Modular code for ease of maintenance
%%%%%%%%%%%%%%%%%%%%

% Add all folders and sub-folders from this level
addpath(genpath(cd))

clear; clc;

% Define Parameters

settings = struct;
settings.chassis.mass = 262;  % vehicle mass (kg)
settings.chassis.weightDistribution = 0.5095;
settings.chassis.massFront = settings.chassis.mass * settings.chassis.weightDistribution;
settings.chassis.wheelRadius = 0.2032;
settings.chassis.wheelbase = 1.531;
settings.chassis.cgHeight =  0.256;
settings.chassis.yawInertia = 106;
settings.aero.rho = 1.196;  % air density (kg/m^3)
settings.aero.frontalArea = 1.157757;  % frontal area (m^2)
settings.aero.CLc = 3.782684;
settings.aero.CDc = 1.410518;
settings.aero.CLs = 4.116061;
settings.aero.CDs = 1.54709;
settings.aero.aeroBalance = 0.531;
settings.powertrain.max_rpm = 5500;
settings.powertrain.FDR = 3.36;
settings.powertrain.powerLimit = 80; % max power [kW]
settings.powertrain.max_speed = (settings.powertrain.max_rpm/settings.powertrain.FDR)*pi*2*settings.chassis.wheelRadius/60;

settings.bounds.maxDelta = deg2rad(25);  % maximum steering angle (rad)
settings.bounds.maxDpsi = deg2rad(120);
settings.bounds.maxBeta = deg2rad(5);
settings.bounds.maxSa = deg2rad(10);
settings.bounds.maxSxf = 0.1;
settings.bounds.maxSxr = 0.1;

% IPOPT Settings
settings.IPOPT.p_opts.print_time = 0;
settings.IPOPT.s_opts.print_level = 0;


% % Mesh Discretization
GGV_settings = struct;
GGV_settings.Vnum = 30;        % number of speed variations
GGV_settings.Gnum = 10;        % number of combine ax/ay variations
GGV_settings.velocityRange = linspace(10,settings.powertrain.max_speed-5, GGV_settings.Vnum); % Discrete Velocity Points

% Select Track
track = load('Track Model\25 Endurance.mat');
settings.track.bContinuousLap = true;

% Calculate GGV Performance Envelope
GGV = TwoWheel.calculateGGV(settings, GGV_settings);

%% Run LapSim

results = runLapSim(GGV, track, settings);

%% Visualise Results
figure("Name",'Car States'); tiledlayout(3,1)
nexttile
plot(results.sLap, results.vCar)
ylim padded
ylabel('vCar [m/s]')

nexttile
plot(results.sLap, results.ax)
ylim padded
ylabel('ax [m/s^2]')

nexttile
plot(results.sLap, results.ay)
ylim padded
ylabel('ay [m/s^2]')
xlabel('sLap [m]')


figure();
scatter3(GGV.gLat, GGV.gLong, GGV.vCar)
hold on
scatter3(results.ay, results.ax, results.vCar)
legend('GGV','LapSim')
legend Location northeast
xlabel('ay [m/s^2]')
ylabel('ax [m/s^2')
zlabel('vCar [m/s]')



% Visualise Results