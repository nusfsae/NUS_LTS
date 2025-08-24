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

% % Tyres
% settings.Tyre.p0  = 84000        ;  % Nominal pressure in Pa [Not used in MF5.2]   
% settings.Tyre.FZ0 = 1080; % Nominal Load in [N]             
% % Scaling Coefficients
% settings.Tyre.LFZO                         = 1;  
% settings.Tyre.LGAY                         = 1;
% settings.Tyre.LCX                          = 1;          
% settings.Tyre.LMUX                         = 1;          
% settings.Tyre.LEX                          = 1;
% settings.Tyre.LKX                          = 1;
% settings.Tyre.LHX                          = 1;
% settings.Tyre.LVX                          = 1;
% settings.Tyre.LCY                          = 1;
% settings.Tyre.LGX                          = 1;
% settings.Tyre.LMUY                         = 1;
% settings.Tyre.LEY                          = 1;
% settings.Tyre.LKY                          = 1;
% settings.Tyre.LHY                          = 1;
% settings.Tyre.LVY                          = 1;
% settings.Tyre.LTR                          = 1;
% settings.Tyre.LRES                         = 1;
% settings.Tyre.LXAL                         = 1;
% settings.Tyre.LYKA                         = 1;
% settings.Tyre.LVYKA                        = 1;
% settings.Tyre.LS                           = 1;
% settings.Tyre.LKYC                         = 1;
% settings.Tyre.LKZC                         = 1;
% settings.Tyre.LVMX                         = 1;
% settings.Tyre.LMX                          = 1;
% settings.Tyre.LMY                          = 1;
% settings.Tyre.LMP                          = 1;
% % longitudinal coefficients
% settings.Tyre.PCX1                         = 1.5               ;
% settings.Tyre.PDX1                         = 2.4722            ;
% settings.Tyre.PDX2                         = -0.78691          ;
% settings.Tyre.PDX3                         = 15                ;
% settings.Tyre.PEX1                         = -2.5811e-13       ;
% settings.Tyre.PEX2                         = -0.87477          ;
% settings.Tyre.PEX3                         = -0.6              ;
% settings.Tyre.PEX4                         = 0.9               ;
% settings.Tyre.PKX1                         = 42.8193           ;
% settings.Tyre.PKX2                         = -0.0001749        ;
% settings.Tyre.PKX3                         = -0.49011          ;
% settings.Tyre.PHX1                         = 0.00093775        ;
% settings.Tyre.PHX2                         = -0.0013228        ;
% settings.Tyre.PVX1                         = -0.02779          ;
% settings.Tyre.PVX2                         = 0.089387          ;
% settings.Tyre.PPX1                         = -1.0177           ;
% settings.Tyre.PPX2                         = -1.3151           ;
% settings.Tyre.PPX3                         = -0.2709           ;
% settings.Tyre.PPX4                         = 0.81854           ;
% settings.Tyre.RBX1                         = 5                 ;
% settings.Tyre.RBX2                         = 5                 ;
% settings.Tyre.RBX3                         = 0                 ;
% settings.Tyre.RCX1                         = 1                 ;
% settings.Tyre.REX1                         = -1                ;
% settings.Tyre.REX2                         = -0.1              ;
% settings.Tyre.RHX1                         = 0                 ;
% % Lateral Coefficients
% settings.Tyre.PCY1                         = 1.5               ;
% settings.Tyre.PDY1                         = 2.5764            ;
% settings.Tyre.PDY2                         = -0.47966          ;
% settings.Tyre.PDY3                         = 1.2505            ;
% settings.Tyre.PEY1                         = 0.44562           ;
% settings.Tyre.PEY2                         = -0.15927          ;
% settings.Tyre.PEY3                         = 0.055312          ;
% settings.Tyre.PEY4                         = 11.0271           ;
% settings.Tyre.PEY5                         = 166.589           ;
% settings.Tyre.PKY1                         = -34.4974          ;
% settings.Tyre.PKY2                         = 1.369             ;
% settings.Tyre.PKY3                         = 0.60632           ;
% settings.Tyre.PKY4                         = 2                 ;
% settings.Tyre.PKY5                         = 83.7446           ;
% settings.Tyre.PKY6                         = -4.1081           ;
% settings.Tyre.PKY7                         = -0.79828          ;
% settings.Tyre.PHY1                         = 0.0036805         ;
% settings.Tyre.PHY2                         = 0.0016442         ;
% settings.Tyre.PHY3                         = 0.1416            ;           % Missing from .Tir file, filled from heuristics
% settings.Tyre.PVY1                         = 0.067909          ;
% settings.Tyre.PVY2                         = 0.016368          ;
% settings.Tyre.PVY3                         = 0.52062           ;
% settings.Tyre.PVY4                         = -3.3053           ;
% settings.Tyre.PPY1                         = 0.42092           ;
% settings.Tyre.PPY2                         = 1.1945            ;
% settings.Tyre.PPY3                         = -0.33642          ;
% settings.Tyre.PPY4                         = -0.52307          ;
% settings.Tyre.PPY5                         = -1.0699           ;
% settings.Tyre.RBY1                         = 5                 ;
% settings.Tyre.RBY2                         = 2                 ;
% settings.Tyre.RBY3                         = 0.02              ;
% settings.Tyre.RBY4                         = 0                 ;
% settings.Tyre.RCY1                         = 1                 ;
% settings.Tyre.REY1                         = -0.1              ;
% settings.Tyre.REY2                         = 0.1               ;
% settings.Tyre.RHY1                         = 0                 ;
% settings.Tyre.RHY2                         = 0                 ;
% settings.Tyre.RVY1                         = 0                 ;
% settings.Tyre.RVY2                         = 0                 ;
% settings.Tyre.RVY3                         = 0                 ;
% settings.Tyre.RVY4                         = 0                 ;
% settings.Tyre.RVY5                         = 0                 ;
% settings.Tyre.RVY6                         = 0                 ;

% OG  Tyres
settings.Tyre.FZ0 = 1100;
settings.Tyre.LFZO = 1.2;
% Scaling Factors

settings.Tyre.LGAY    =   1;
settings.Tyre.LHY     =   1;
settings.Tyre.LVY     =   1;
settings.Tyre.LCY     =   1;
settings.Tyre.LEY     =   1;
settings.Tyre.LHX     =   1;
settings.Tyre.LVX     =   1;
settings.Tyre.LGX     =   1;
settings.Tyre.LCX     =   1;
settings.Tyre.LEX     =   1;
settings.Tyre.LXAL    =   1;

settings.Tyre.LKY     =   1; % 1
settings.Tyre.LKX     =   1; %  0.7
settings.Tyre.LMUY    =   1; %0.5; % 0.38 Changed to Fit
settings.Tyre.LMUX    =   1; % 0.5; % 0.25 Changed to Fit

% Longitudinal Coefficients
settings.Tyre.PCX1   =   1.2602;
settings.Tyre.PDX1   =   2.354;
settings.Tyre.PDX2   =   -0.015401;
settings.Tyre.PDX3   =   -0.76992;
settings.Tyre.PEX1   =  -1.0845;
settings.Tyre.PEX2   =   2.3203;
settings.Tyre.PEX3   =   3.2136;
settings.Tyre.PEX4   =   -1.7027;
settings.Tyre.PKX1   =   39.334;
settings.Tyre.PKX2   =   -0.37146;
settings.Tyre.PKX3   =   0.37752;
settings.Tyre.PHX1   =   0.025058;
settings.Tyre.PHX2   =   -0.038843;
settings.Tyre.PVX1   =   -0.00045953;
settings.Tyre.PVX2   =   0.0013401;

% Combined Longitudinal Coefficients
settings.Tyre.RBX1   =   7.4574;
settings.Tyre.RBX2   =   -8.8044;
settings.Tyre.RCX1   =   1.5974;
settings.Tyre.REX1   =   0.22918;
settings.Tyre.REX2   =   -0.5217;
settings.Tyre.RHX1   =   0;

% Lateral Coefficients
settings.Tyre.PCY1   =   1.4;
settings.Tyre.PDY1   =   2.4;
settings.Tyre.PDY2   =   -0.4507889;
settings.Tyre.PDY3   =   20;
settings.Tyre.PEY1   =   0.01;
settings.Tyre.PEY2   =   0.05;
settings.Tyre.PEY3   =   10;
settings.Tyre.PEY4   =   0;
settings.Tyre.PKY1   =   -27.3678;
settings.Tyre.PKY2   =   1.242483;
settings.Tyre.PKY3   =   3;
settings.Tyre.PHY1   =   -0.00002845241;
settings.Tyre.PHY2   =   -0.0000329537;
settings.Tyre.PHY3   =   0.1416031;
settings.Tyre.PVY1   =   0;
settings.Tyre.PVY2   =   -0.009009;
settings.Tyre.PVY3   =   -0.5;
settings.Tyre.PVY4   =   -1;

% Combined Lateral Coefficients
settings.Tyre.RBY1   =   26.3099;
settings.Tyre.RBY2   =   20.3304;
settings.Tyre.RBY3   =   -0.015204;
settings.Tyre.RCY1   =   0.96889;
settings.Tyre.REY1   =   0.53522;
settings.Tyre.REY2   =   0.69602;
settings.Tyre.RHY1   =   0;
settings.Tyre.RHY2   =   0;
settings.Tyre.RVY1   =   0;
settings.Tyre.RVY2   =   0;
settings.Tyre.RVY3   =   0;
settings.Tyre.RVY4   =   0;
settings.Tyre.RVY5   =   0;
settings.Tyre.RVY6   =   0;

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
        inputs.delta = opti.variable();            opti.subject_to(-settings.bounds.maxDelta<=settings.bounds.maxDelta);             % steering angle (rad)
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
