% Main Script to Run Quasi Steady State LapSim - 4 Wheel Model

%%%%%%%%%%%%%%%%%%%%
% 25/08/2025
% Objectives:
% 1. Legible Code Flow
% 2. Clean workspace for debugging
% 3. Modular code for ease of maintenance
%%%%%%%%%%%%%%%%%%%%
clear; clc;

% Add all folders and sub-folders from this level
addpath(genpath(cd))

% Settings
settings = struct;
g = 9.81;

% Chassis Properties
settings.Chassis.mass = 262;
settings.Chassis.unsprungMass = 10; % Abritrary, to be updated
settings.Chassis.SprungMass = settings.Chassis.mass - 4*(settings.Chassis.unsprungMass);
settings.Chassis.heightUnsprungCOG = 0.2032; % Arbitrary, to be updated
settings.Chassis.heightSprungCOG = 0.256;
settings.Chassis.weightDist = 0.5;
settings.Chassis.radWheel = 0.2032; % Loaded radius
settings.Chassis.wheelBase = 1.531; % Isn't the minimum 1530mm?
settings.Chassis.trackWidth = 1.21;
settings.Chassis.massFront = settings.Chassis.mass*settings.Chassis.weightDist;
settings.Chassis.massRear = settings.Chassis.mass*(1-settings.Chassis.weightDist);
settings.Chassis.sprungMassFront = settings.Chassis.SprungMass*settings.Chassis.weightDist;
settings.Chassis.sprungMassRear = settings.Chassis.SprungMass*(1-settings.Chassis.weightDist);
settings.Chassis.frontMomentArm = settings.Chassis.wheelBase*(1 - settings.Chassis.weightDist);
settings.Chassis.rearMomentArm = settings.Chassis.wheelBase*(settings.Chassis.weightDist);
settings.Chassis.yawInertia = 106; % kgm^2

% Aerodynamic Properties
settings.Aero.ClA = 4.11;
settings.Aero.CdA = 1.54;
settings.Aero.rAeroBalance = 0.531;
settings.Aero.ClAf = settings.Aero.ClA*settings.Aero.rAeroBalance;
settings.Aero.ClAr = settings.Aero.ClA*(1-settings.Aero.rAeroBalance);

% Suspension Properties
settings.Suspension.heightCG2rollAxis = 0.230; % Arbitrary
settings.Suspension.rollCentreFront = 0.035; % Arbitrary
settings.Suspension.rollCentreRear = 0.035; % Arbitrary
settings.Suspension.mechanicalBalance = 0.5; % Arbitrary
% Suspension Setup   
settings.Suspension.aCamberFront = 0; % [deg], negative is leaning inwards
settings.Suspension.aCamberRear = 0; % [deg], negative is leaning inwards
settings.Suspension.aToeStaticFront = 0; % [deg], negative is toe inwards, wheel pointing inwards to chassis
settings.Suspension.aToeStaticRear = 0; % [deg], negative is toe inwards, wheel pointing inwards to chassis

% Brake Properties
settings.Brakes.muBrakePad = 0.4;
settings.Brakes.nPistonFront = 4;
settings.Brakes.nPistonRear = 2;
settings.Brakes.diamPiston = 0.0254;
settings.Brakes.areaPiston = (0.25*pi*settings.Brakes.diamPiston^2);
settings.Brakes.radBrakeDisc = 0.1836/2;
settings.Brakes.rBrakeBias = 0.55;

% Tyres
settings = Tyres.Hoosier16x75(settings); % 16x7.5in Tyres
% settings = Tyres.Hoosier16x6(settings); % Reference - 16x6in Tyres

% Powertrain Properties
settings.Powertrain.MaxPower = 80; % [kW], Power Limit
settings.Powertrain.max_rpm = 5500;
settings.Powertrain.FDR = 3.36;
settings.Powertrain.max_speed = (settings.Powertrain.max_rpm/settings.Powertrain.FDR)*pi*2*settings.Chassis.radWheel/60;

% Bounds
settings.bounds.maxDelta = deg2rad(25);  % maximum steering angle (rad)
settings.bounds.maxBeta = deg2rad(5);
settings.bounds.maxSa = deg2rad(10);
settings.bounds.minSxf = -0.15;
settings.bounds.maxSxf = 0; % No tractive force at front axle for RWD
settings.bounds.minSxr = -0.15;
settings.bounds.maxSxr = 0.15;

% IPOPT Settings
settings.IPOPT.p_opts.print_time = 0;
settings.IPOPT.s_opts.print_level = 0;

% Mesh Discretisation for GGV diagram
GGV_settings = struct;
GGV_settings.Vnum = 10;        % number of speed variations
GGV_settings.Gnum = 20;        % number of combine ax/ay variations
GGV_settings.velocityRange = linspace(10,settings.Powertrain.max_speed-5, GGV_settings.Vnum); % Discrete Velocity Points

% Calculate GGV Envelope
GGV = FourWheel.calculateGGV(settings, GGV_settings);

%% Select Track
track = load('Track Model\25 Endurance.mat');
settings.track.bContinuousLap = true;

% Run LapSim
results = runLapSim(GGV, track, settings);

% Visualise Results
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