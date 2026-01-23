%% parameters (OPTION 1: plain doubles, Simulink-friendly in MATLAB Function blocks)

% Chassis Settings
chassis.car       = 199.6;     % vehicle mass (kg)
chassis.driver    = 65;        % driver mass (kg)
chassis.track     = 1.21;      % track width (m)
chassis.cg_f      = 0.5095;    % mass bias to front (-)
chassis.wheelbase = 1.531;     % wheelbase (m)
chassis.cg_h      = 0.256;     % CG height (m)

% Suspension Settings
suspension.del_max   = 0.565;   % maximum steering angle (rad)
suspension.R         = 0.2032;  % wheel radius (m)
suspension.P         = 9;       % tire pressure (psi)
suspension.IA        = 0;       % inclination angle (rad)  (note: you later treat IA as deg in MF52)
suspension.brakebias = 0.67;    % brake bias (-)

% Aerodynamics Settings
aero.den   = 1.196;      % air density (kg/m^3)
aero.farea = 1.157757;   % frontal area (m^2)
aero.CLc   = 3.828;      % CL cornering
aero.CDc   = 1.403;      % CD cornering
aero.ab_c  = 0.528;      % aero balance cornering (front)
aero.CLs   = 4.034;      % CL straight line
aero.CDs   = 1.543;      % CD straight line
aero.ab_s  = 0.549;      % aero balance straight line (front)

% Powertrain Settings
pwt.max_rpm   = 5500;    % maximum wheel speed (rpm)
pwt.FDR       = 3.36;    % final drive ratio (-)
pwt.Ipeak     = 1;       % power percentage (-)
pwt.PMaxLimit = 80;      % power limit (kW)

% Derived / 2-step calculation
chassis.mass = chassis.car + chassis.driver;  % total mass (kg)

pwt.v_max = (pwt.max_rpm / pwt.FDR) * (2*pi) * suspension.R / 60;  % maximum speed (m/s)


%% tyre parameters (H16x7.5-R20)  (OPTION 1: plain doubles)

% Nominal Conditions
tyre.p0  = 84000;   % nominal pressure (Pa)
tyre.FZ0 = 1080;    % nominal load (N)

% Scaling Coefficients
tyre.LFZO  = 1;
tyre.LGAY  = 1;
tyre.LCX   = 1;
tyre.LMUX  = 1;
tyre.LEX   = 1;
tyre.LKX   = 1;
tyre.LHX   = 1;
tyre.LVX   = 0;      % set to 0 due to oddities
tyre.LCY   = 1;
tyre.LGX   = 1;
tyre.LMUY  = 1;
tyre.LEY   = 1;
tyre.LKY   = 1;
tyre.LHY   = 1;
tyre.LVY   = 1;
tyre.LTR   = 1;
tyre.LRES  = 1;
tyre.LXAL  = 1;
tyre.LYKA  = 1;
tyre.LVYKA = 1;
tyre.LS    = 1;
tyre.LKYC  = 1;
tyre.LKZC  = 1;
tyre.LVMX  = 1;
tyre.LMX   = 1;
tyre.LMY   = 1;
tyre.LMP   = 1;

% Longitudinal Coefficients
tyre.PCX1 = 1.5;
tyre.PDX1 = 2.4722;
tyre.PDX2 = -0.78691;
tyre.PDX3 = 15;
tyre.PEX1 = -2.5811e-13;
tyre.PEX2 = -0.87477;
tyre.PEX3 = -0.6;
tyre.PEX4 = 0.9;
tyre.PKX1 = 42.8193;
tyre.PKX2 = -0.0001749;
tyre.PKX3 = -0.49011;
tyre.PHX1 = 0.00093775;
tyre.PHX2 = -0.0013228;
tyre.PVX1 = -0.02779;
tyre.PVX2 = 0.089387;
tyre.PPX1 = -1.0177;
tyre.PPX2 = -1.3151;
tyre.PPX3 = -0.2709;
tyre.PPX4 = 0.81854;
tyre.RBX1 = 5;
tyre.RBX2 = 5;
tyre.RBX3 = 0;
tyre.RCX1 = 1;
tyre.REX1 = -1;
tyre.REX2 = -0.1;
tyre.RHX1 = 0;

% Lateral Coefficients
tyre.PCY1 = 1.5;
tyre.PDY1 = 2.5764;
tyre.PDY2 = -0.47966;
tyre.PDY3 = 1.2505;
tyre.PEY1 = 0.44562;
tyre.PEY2 = -0.15927;
tyre.PEY3 = 0.055312;
tyre.PEY4 = 11.0271;
tyre.PEY5 = 166.589;
tyre.PKY1 = -34.4974;
tyre.PKY2 = 1.369;
tyre.PKY3 = 0.60632;
tyre.PKY4 = 2;
tyre.PKY5 = 83.7446;
tyre.PKY6 = -4.1081;
tyre.PKY7 = -0.79828;
tyre.PHY1 = 0.0036805;
tyre.PHY2 = 0.0016442;
tyre.PHY3 = 0.1416;   % filled heuristically
tyre.PVY1 = 0.067909;
tyre.PVY2 = 0.016368;
tyre.PVY3 = 0.52062;
tyre.PVY4 = -3.3053;
tyre.PPY1 = 0.42092;
tyre.PPY2 = 1.1945;
tyre.PPY3 = -0.33642;
tyre.PPY4 = -0.52307;
tyre.PPY5 = -1.0699;
tyre.RBY1 = 5; 
tyre.RBY2 = 2;
tyre.RBY3 = 0.02;
tyre.RBY4 = 0;
tyre.RCY1 = 1;
tyre.REY1 = -0.1;
tyre.REY2 = 0.1;
tyre.RHY1 = 0;
tyre.RHY2 = 0;
tyre.RVY1 = 0;
tyre.RVY2 = 0;
tyre.RVY3 = 0;
tyre.RVY4 = 0;
tyre.RVY5 = 0;
tyre.RVY6 = 0;




% %% parameters
% 
% % Chassis Settings
% chassis.car      = Simulink.Parameter(199.6);   % vehicle mass (kg)
% chassis.driver   = Simulink.Parameter(65);      % driver mass (kg)
% chassis.track    = Simulink.Parameter(1.21);    % track width (m)
% chassis.cg_f     = Simulink.Parameter(0.5095);  % mass bias to front (-)
% chassis.wheelbase= Simulink.Parameter(1.531);   % wheelbase (m)
% chassis.cg_h     = Simulink.Parameter(0.256);   % CG height (m)
% 
% % Suspension Settings
% suspension.del_max   = Simulink.Parameter(0.565);   % max steering angle (rad)
% suspension.R         = Simulink.Parameter(0.2032);  % wheel radius (m)
% suspension.P         = Simulink.Parameter(9);       % tire pressure (psi)
% suspension.IA        = Simulink.Parameter(0);       % inclination angle (rad)
% suspension.brakebias = Simulink.Parameter(0.67);    % brake bias (-)
% 
% % Aerodynamics Settings
% aero.den   = Simulink.Parameter(1.196);      % air density (kg/m^3)
% aero.farea = Simulink.Parameter(1.157757);   % frontal area (m^2)
% aero.CLc   = Simulink.Parameter(3.828);      % CL cornering
% aero.CDc   = Simulink.Parameter(1.403);      % CD cornering
% aero.ab_c  = Simulink.Parameter(0.528);      % aero balance cornering (front)
% aero.CLs   = Simulink.Parameter(4.034);      % CL straight line
% aero.CDs   = Simulink.Parameter(1.543);      % CD straight line
% aero.ab_s  = Simulink.Parameter(0.549);      % aero balance straight line (front)
% 
% % Powertrain Settings
% pwt.max_rpm   = Simulink.Parameter(5500);  % max wheel speed (rpm)
% pwt.FDR       = Simulink.Parameter(3.36);  % final drive ratio (-)
% pwt.Ipeak     = Simulink.Parameter(1);     % power percentage (-)
% pwt.PMaxLimit = Simulink.Parameter(80);    % power limit (kW)
% 
% % Derived parameters (use .Value for arithmetic)
% chassis.mass = Simulink.Parameter(chassis.car.Value + chassis.driver.Value);
% 
% pwt.v_max = Simulink.Parameter( ...
%     (pwt.max_rpm.Value / pwt.FDR.Value) * (2*pi) * suspension.R.Value / 60 ...
% ); % maximum speed (m/s)
% 
% 
% %% Tyre Settings – Hoosier 16x7.5
% 
% % Nominal Conditions
% tyre.p0  = Simulink.Parameter(84000);     % nominal pressure (Pa)
% tyre.FZ0 = Simulink.Parameter(1080);      % nominal load (N)
% 
% % Scaling Coefficients
% tyre.LFZO  = Simulink.Parameter(1);
% tyre.LGAY  = Simulink.Parameter(1);
% tyre.LCX   = Simulink.Parameter(1);
% tyre.LMUX  = Simulink.Parameter(1);
% tyre.LEX   = Simulink.Parameter(1);
% tyre.LKX   = Simulink.Parameter(1);
% tyre.LHX   = Simulink.Parameter(1);
% tyre.LVX   = Simulink.Parameter(0);       % set to 0 due to oddities
% tyre.LCY   = Simulink.Parameter(1);
% tyre.LGX   = Simulink.Parameter(1);
% tyre.LMUY  = Simulink.Parameter(1);
% tyre.LEY   = Simulink.Parameter(1);
% tyre.LKY   = Simulink.Parameter(1);
% tyre.LHY   = Simulink.Parameter(1);
% tyre.LVY   = Simulink.Parameter(1);
% tyre.LTR   = Simulink.Parameter(1);
% tyre.LRES  = Simulink.Parameter(1);
% tyre.LXAL  = Simulink.Parameter(1);
% tyre.LYKA  = Simulink.Parameter(1);
% tyre.LVYKA = Simulink.Parameter(1);
% tyre.LS    = Simulink.Parameter(1);
% tyre.LKYC  = Simulink.Parameter(1);
% tyre.LKZC  = Simulink.Parameter(1);
% tyre.LVMX  = Simulink.Parameter(1);
% tyre.LMX   = Simulink.Parameter(1);
% tyre.LMY   = Simulink.Parameter(1);
% tyre.LMP   = Simulink.Parameter(1);
% 
% % Longitudinal Coefficients
% tyre.PCX1 = Simulink.Parameter(1.5);
% tyre.PDX1 = Simulink.Parameter(2.4722);
% tyre.PDX2 = Simulink.Parameter(-0.78691);
% tyre.PDX3 = Simulink.Parameter(15);
% tyre.PEX1 = Simulink.Parameter(-2.5811e-13);
% tyre.PEX2 = Simulink.Parameter(-0.87477);
% tyre.PEX3 = Simulink.Parameter(-0.6);
% tyre.PEX4 = Simulink.Parameter(0.9);
% tyre.PKX1 = Simulink.Parameter(42.8193);
% tyre.PKX2 = Simulink.Parameter(-0.0001749);
% tyre.PKX3 = Simulink.Parameter(-0.49011);
% tyre.PHX1 = Simulink.Parameter(0.00093775);
% tyre.PHX2 = Simulink.Parameter(-0.0013228);
% tyre.PVX1 = Simulink.Parameter(-0.02779);
% tyre.PVX2 = Simulink.Parameter(0.089387);
% tyre.PPX1 = Simulink.Parameter(-1.0177);
% tyre.PPX2 = Simulink.Parameter(-1.3151);
% tyre.PPX3 = Simulink.Parameter(-0.2709);
% tyre.PPX4 = Simulink.Parameter(0.81854);
% tyre.RBX1 = Simulink.Parameter(5);
% tyre.RBX2 = Simulink.Parameter(5);
% tyre.RBX3 = Simulink.Parameter(0);
% tyre.RCX1 = Simulink.Parameter(1);
% tyre.REX1 = Simulink.Parameter(-1);
% tyre.REX2 = Simulink.Parameter(-0.1);
% tyre.RHX1 = Simulink.Parameter(0);
% 
% % Lateral Coefficients
% tyre.PCY1 = Simulink.Parameter(1.5);
% tyre.PDY1 = Simulink.Parameter(2.5764);
% tyre.PDY2 = Simulink.Parameter(-0.47966);
% tyre.PDY3 = Simulink.Parameter(1.2505);
% tyre.PEY1 = Simulink.Parameter(0.44562);
% tyre.PEY2 = Simulink.Parameter(-0.15927);
% tyre.PEY3 = Simulink.Parameter(0.055312);
% tyre.PEY4 = Simulink.Parameter(11.0271);
% tyre.PEY5 = Simulink.Parameter(166.589);
% tyre.PKY1 = Simulink.Parameter(-34.4974);
% tyre.PKY2 = Simulink.Parameter(1.369);
% tyre.PKY3 = Simulink.Parameter(0.60632);
% tyre.PKY4 = Simulink.Parameter(2);
% tyre.PKY5 = Simulink.Parameter(83.7446);
% tyre.PKY6 = Simulink.Parameter(-4.1081);
% tyre.PKY7 = Simulink.Parameter(-0.79828);
% tyre.PHY1 = Simulink.Parameter(0.0036805);
% tyre.PHY2 = Simulink.Parameter(0.0016442);
% tyre.PHY3 = Simulink.Parameter(0.1416);    % filled heuristically
% tyre.PVY1 = Simulink.Parameter(0.067909);
% tyre.PVY2 = Simulink.Parameter(0.016368);
% tyre.PVY3 = Simulink.Parameter(0.52062);
% tyre.PVY4 = Simulink.Parameter(-3.3053);
% tyre.PPY1 = Simulink.Parameter(0.42092);
% tyre.PPY2 = Simulink.Parameter(1.1945);
% tyre.PPY3 = Simulink.Parameter(-0.33642);
% tyre.PPY4 = Simulink.Parameter(-0.52307);
% tyre.PPY5 = Simulink.Parameter(-1.0699);
% tyre.RBY1 = Simulink.Parameter(5);
% tyre.RBY2 = Simulink.Parameter(2);
% tyre.RBY3 = Simulink.Parameter(0.02);
% tyre.RBY4 = Simulink.Parameter(0);
% tyre.RCY1 = Simulink.Parameter(1);
% tyre.REY1 = Simulink.Parameter(-0.1);
% tyre.REY2 = Simulink.Parameter(0.1);
% tyre.RHY1 = Simulink.Parameter(0);
% tyre.RHY2 = Simulink.Parameter(0);
% tyre.RVY1 = Simulink.Parameter(0);
% tyre.RVY2 = Simulink.Parameter(0);
% tyre.RVY3 = Simulink.Parameter(0);
% tyre.RVY4 = Simulink.Parameter(0);
% tyre.RVY5 = Simulink.Parameter(0);
% tyre.RVY6 = Simulink.Parameter(0);
