addpath('C:\Users\PC5\Documents\casadi-3.6.7-windows64-matlab2018b')
addpath(genpath(cd))
import casadi.*

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
CDs = 1.54709;                       % CD cornering
ab = 0.5310665;                      % aero balance (front)
% Powertrain Settings
max_rpm = 5500;                      % maximum wheel speed (rpm)
FDR = 3.36;                          % final drive ratio (-)
Ipeak = 1;                           % power percentage (-)               
v_max = (max_rpm/FDR)*pi*2*R/60;     % maximum speed (m/s)
PMaxLimit = 80;                      % power limit (KW)

% % Bounds for Path Constraints
maxp = 100;                           % maximum radius of GG diagram (m/s^2)
maxDelta = del_max;                  % maximum steering angle (rad)
maxSa = deg2rad(10);                 % maximum slip angle (deg)
maxBeta = deg2rad(20);               % maximum body slip (deg)
maxSxfr = 0.1;                       % maximum front right slip ratio (-)
maxSxfl = 0.1;                       % maximum front left slip ratio (-)
maxSxrr = 0.1;                       % maximum rear right slip ratio (-)
maxSxrl = 0.1;                       % maximum rear left slip ratio (-)
maxDpsi = deg2rad(90);              % maximum yaw rate (deg/s)

% % IPOPT Settings
opts = struct();
opts.print_time = false;
opts.ipopt.print_level = 5;
opts.ipopt.tol = 1e-6;
opts.ipopt.acceptable_tol = 1e-4;           
opts.ipopt.acceptable_iter = 15;
opts.ipopt.max_iter = 3000;
% opts.ipopt.mu_strategy = 'adaptive';
% opts.ipopt.linear_solver = 'ma57';
% opts.ipopt.hessian_approximation = 'limited-memory';

% % Mesh Discretization
Gnum = 20;       

tic
% % Create empty performance envelope GG
GG = struct();

%%
figure

% % Steady State Speed Setting
V = 15; 
% empty array for ay
GG.ay = zeros(1, Gnum);
% Range of ax/ay combinations
AngleRange = linspace(pi/2,-pi/2,Gnum);
% each combination of ax/ay
for j = 1:numel(AngleRange)
    % ax/ay angle
    theta = AngleRange(j);
    % call solver
    opti = casadi.Opti();
    % input: decision variables
    p = opti.variable(); opti.subject_to(0<=p<=maxp);                           % radius of GGD (ms^-2)
    delta = opti.variable(); opti.subject_to(-maxDelta<=delta<=maxDelta);       % steering angle (rad)
    beta = opti.variable(); opti.subject_to(-maxBeta<=beta<=maxBeta);           % body slip (rad)
    Sxfl = opti.variable(); opti.subject_to(-maxSxfl<=Sxfl<=0);                 % front left slip ratio
    Sxfr = opti.variable(); opti.subject_to(-maxSxfr<=Sxfr<=0);                 % front right slip ratio
    Sxrl = opti.variable(); opti.subject_to(-maxSxrl<=Sxrl<=maxSxrl);           % rear left slip ratio
    Sxrr = opti.variable(); opti.subject_to(-maxSxrr<=Sxrr<=maxSxrr);           % rear right slip ratio
    dpsi = opti.variable(); opti.subject_to(-maxDpsi<=dpsi<=maxDpsi);           % Yaw rate (rad/s)
    % inputs: additional
    ax_in = p*sin(theta);
    ay_in = p*cos(theta);
    % call Vehicle Model
    vehicle;
    % define initial guess
    opti.set_initial(delta,0);
    opti.set_initial(beta,0);
    opti.set_initial(Sxfl,0);
    opti.set_initial(Sxfr,0);
    opti.set_initial(Sxrl,0);
    opti.set_initial(Sxrr,0);
    opti.set_initial(dpsi,0);
    % closer initial guess
    if j>1
        opti.set_initial(delta,GG.delta(j-1));
        opti.set_initial(beta,GG.beta(j-1));
        opti.set_initial(Sxfl,GG.Sxfl(j-1));
        opti.set_initial(Sxfr,GG.Sxfr(j-1));
        opti.set_initial(Sxrl,GG.Sxrl(j-1));
        opti.set_initial(Sxrr,GG.Sxrr(j-1));
        opti.set_initial(dpsi,GG.dpsi(j-1));
    end
    % define constraints
    opti.subject_to(ax_res ==0);
    opti.subject_to(ay_res ==0);
    opti.subject_to(Mz == 0);
    opti.subject_to(ay - V*dpsi == 0);
    opti.subject_to(-maxSa<=Safl<=maxSa);
    opti.subject_to(-maxSa<=Safr<=maxSa);
    opti.subject_to(-maxSa<=Sarl<=maxSa);
    opti.subject_to(-maxSa<=Sarr<=maxSa);
    % optimization results
    opti.solver('ipopt', opts);
    % objective
    opti.minimize(-p);
    % results
    x = opti.solve();
    GG.ax(j) = x.value(ax);
    GG.ay(j) = x.value(ay);
    GG.delta(j) = x.value(delta);
    GG.beta(j) = x.value(beta);
    GG.dpsi(j) = x.value(dpsi);
    % slip ratio
    GG.Sxfl(j) = x.value(Sxfl);
    GG.Sxfr(j) = x.value(Sxfr);
    GG.Sxrl(j) = x.value(Sxrl);
    GG.Sxrr(j) = x.value(Sxrr);
    % slip angle
    GG.Safl(j) = x.value(Safl);
    GG.Safr(j) = x.value(Safr);
    GG.Sarl(j) = x.value(Sarl);
    GG.Sarr(j) = x.value(Sarr);
    % real-time plot
    plot(GG.ay(j),GG.ax(j),'x')
    hold on
end


figure
plot(GG.ay,GG.ax)
% figure
% yyaxis left
% plot(rad2deg(GG.delta))
% yyaxis right
% plot(rad2deg(GG.beta))
% figure
% plot(GG.delta)
% figure
% plot(GG.beta)
% figure
% plot(GG.dpsi)
% figure
% plot(GG.Sxf)
% figure
% plot(GG.Sxr)

toc



