%% Main Script to Calculate GGV Diagram

clear; clc;

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

%% Define Speed Range to calculate GG diagram
% Sequence of GGV calculation
GGV_settings = struct;
GGV_settings.Vnum = 10;        % number of speed variations
GGV_settings.Gnum = 20;        % number of combine ax/ay variations
GGV_settings.velocityRange = linspace(10,settings.Powertrain.max_speed-5, GGV_settings.Vnum); % Discrete Velocity Points


GGV = testCalculateGGV(settings, GGV_settings);

function GGV = testCalculateGGV(settings, GGV_settings)

import casadi.*

for i = 1:numel(GGV_settings.velocityRange)

    velocity = GGV_settings.velocityRange(i);

    % Angle Range
    alphaRange = linspace(-pi/2, pi/2, GGV_settings.Gnum);
    
    tStart = tic;
    
    for j = 1:numel(alphaRange)
        alpha = alphaRange(j);
        
        % Create Optimisation Problem
        opti = casadi.Opti();
        
        % decision variables & box constraints
        inputs = struct;
        
        inputs.p     = opti.variable();            opti.subject_to(0<=inputs.p<=10);                       % Radius of GG envelope (g)
        inputs.delta = opti.variable();            opti.subject_to(-settings.bounds.maxDelta<=inputs.delta<=settings.bounds.maxDelta);             % steering angle (rad)
        inputs.beta = opti.variable();             opti.subject_to(-settings.bounds.maxBeta<=inputs.beta<=settings.bounds.maxBeta);              % sideslip angle (rad)
        inputs.kappa_fl = opti.variable();         opti.subject_to(-0.15<=inputs.kappa_fl<=0)           % FL wheel angular velocity (rad/s)
        inputs.kappa_fr = opti.variable();         opti.subject_to(-0.15<=inputs.kappa_fr<=0)           % FR wheel angular velocity (rad/s)
        inputs.kappa_rl = opti.variable();         opti.subject_to(-0.15<=inputs.kappa_rl<=0.15)           % RL wheel angular velocity (rad/s)
        inputs.kappa_rr = opti.variable();         opti.subject_to(-0.15<=inputs.kappa_rr<=0.15)           % RR wheel angular velocity (rad/s)
        
        % Additional Inputs
        inputs.Vx = velocity;         % Forward Velocity (m/s)
        inputs.ay_control = inputs.p * 9.81 * cos(alpha);        % Lateral Acceleration - imposed as a result of GG envelope radius and angle alpha
        inputs.ax_control = inputs.p * 9.81 * sin(alpha);        % Longitudinal Acceleration - imposed as a result of GG envelope radius and angle alpha
        inputs.curvature = inputs.ay_control / inputs.Vx^2;
        inputs.yawRate = inputs.Vx * inputs.curvature;
    
        outputs = FourWheel.vehicleModel(inputs, settings);    
        
        % Common Constraints
        opti.subject_to(outputs.residuals.ay_res==0);
        opti.subject_to(outputs.residuals.ax_res==0);
        opti.subject_to(outputs.residuals.power<=0);
        opti.subject_to(outputs.Mz==0);
        
        % Common Solver Settings
        plugin_opts = struct('print_time',0);
        solver_opts = struct('print_level',0);
        opti.solver('ipopt', plugin_opts, solver_opts);  
        
        % Objective - Maximum GG Radius for this angle alpha
        opti.minimize(-(inputs.p^2));

        try
            sol = opti.solve(); 
        
            results.velocity(i,j) = velocity;
            results.ax(i,j) = sol.value(outputs.ax);
            results.ay(i,j) = sol.value(outputs.ay);

        catch
            fprintf('Failed at Velocity %0.2f [m/s] and Alpha %0.2f [deg] \n', velocity, rad2deg(alpha))
        end
    
    end
    
    toc(tStart);

end

%%

GGV_outputs = struct;
GGV_outputs.velocity = reshape(results.velocity,[],1);
GGV_outputs.ax       = reshape(results.ax,[],1);
GGV_outputs.ay       = reshape(results.ay,[],1);


figure(1);clf
scatter3(GGV_outputs.ay, GGV_outputs.ax, GGV_outputs.velocity,[],GGV_outputs.velocity)

GGV = GGV_outputs;

end
