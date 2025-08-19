% % Vehicle Model
mass = 262;  % vehicle mass (kg)
den = 1.196;  % air density
farea = 1.157757;  % frontel area (m^2)
CLc = 3.782684;
CDc = 1.410518;
CLs = 4.116061;
CDs = 1.54709;
del_max = 0.565;  % maximum steering angle (rad)
max_rpm = 5500;
FDR = 3.36;
R = 0.2032;
Ipeak = 1;
wheelbase = 1.531;
cg_h = 0.256;
ab = 0.5310665;
mass_front = 0.5095; % mass distribution to front
Inertia = 106;
v_min = 10; % [m/s] minimum speed for GG calculation
v_max = (max_rpm/FDR)*pi*2*R/60; % maximum speed
PMaxLimit = 80; % [kW] Power Limit
IA = 0; % inclination angle (rad)


import casadi.*

% % Bounds for Path Constraints
maxDelta = deg2rad(25); % maximum steering angle (rad)
maxSa = deg2rad(10);
maxBeta = deg2rad(10);
maxSxf = 0.1;
maxSxr = 0.1;
maxDpsi = deg2rad(120); % deg/s to rad/s

% % IPOPT Settings
p_opts = struct;
s_opts = struct;
opts.expand =true;
p_opts.print_time = 0;
s_opts.print_level = 0; % 0: no display, 5: display
% p_opts.ipopt.accept_every_trial_step = true;
% p_opts.ipopt.constr_viol_tol =1e-3; % set tolerance 
% p_opts.ipopt.restoration_phase ='yes'; % disable restoration phase
% p_opts.ipopt.mu_strategy ='adaptive'; % change mu strategy


% % Mesh Discretization
Vnum = 30;        % number of speed variations
Gnum = 10;        % number of combine ax/ay variations
velocityRange = linspace(v_min,v_max-5, Vnum); % Discrete Velocity Points

tic
% % Create empty performance envelope GG
GG = struct();                  % number of speed variations
Gnum = 50;                     % number of longG variations
GG.ax = zeros(1,Gnum);
GG.ay = zeros(1,Gnum);
GG.delta = zeros(1,Gnum);
GG.beta = zeros(1,Gnum);
GG.dpsi = zeros(1,Gnum);
GG.Sxf = zeros(1,Gnum);
GG.Sxr = zeros(1,Gnum);
GG.Saf = zeros(1,Gnum);
GG.Sar = zeros(1,Gnum);


% % Steady State Speed Setting
V = 25; % (m/s)

GG.speed = V;
% Maximum Forward Acceleration
prob = casadi.Opti();
% Initialise Decision Variables
delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);       % steering angle (rad)
beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);           % body slip (rad)
Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);               % front slip ratio
Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);               % rear slip ratio
dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);           % Yaw rate (rad/s)
% Call Vehicle Model
vehicle;
% define initial guess
prob.set_initial(delta,0);
prob.set_initial(beta,0);
prob.set_initial(Sxf,0);
prob.set_initial(Sxr,0);
prob.set_initial(dpsi,0);
if V>10
    prob.set_initial(Sxf,GG.Sxf(1));
    prob.set_initial(Sxr,GG.Sxr(1));
end
% define constraints
% prob.subject_to(PowerOut<=PMaxLimit);
prob.subject_to(Mz == 0);
prob.subject_to(ay - V*dpsi == 0);
prob.solver('ipopt', p_opts, s_opts);

% % acceleration G solver
prob.minimize(-ax);
x = prob.solve();
maxAx = x.value(ax);
GG.delta(1) = x.value(delta);
GG.beta(1) = x.value(beta);
GG.dpsi(1) = x.value(dpsi);
GG.Sxf(1) = x.value(Sxf);
GG.Sxr(1) = x.value(Sxr);
GG.Sar(1) = x.value(Sar);
GG.Saf(1) = x.value(Saf);

% % braking G solver
% redefine initial guess
if V>10
    prob.set_initial(Sxf,GG.Sxf(end));
    prob.set_initial(Sxr,GG.Sxr(end));
end
prob.minimize(ax);
x = prob.solve();
minAx = x.value(ax);
GG.delta(numel(velocityRange)+2) = x.value(delta);
GG.beta(numel(velocityRange)+2) = x.value(beta);
GG.dpsi(numel(velocityRange)+2) = x.value(dpsi);
GG.Sxf(numel(velocityRange)+2) = x.value(Sxf);
GG.Sxr(numel(velocityRange)+2) = x.value(Sxr);
GG.Sar(numel(velocityRange)+2) = x.value(Sar);
GG.Saf(numel(velocityRange)+2) = x.value(Saf);

% % equal spread ax to -ax
GG.ax = [linspace(maxAx, minAx, Gnum), minAx];
GG.ay = zeros(1, numel(GG.ax));

% Lateral G Solver
for j = 1:numel(GG.ax)-1
    ax_target = GG.ax(j);
    prob = casadi.Opti();
    % Decision Variables
    delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);        % steering angle (rad)
    beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
    Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);                % front slip ratio
    Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);                % rear slip ratio
    dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);            % Yaw rate (rad/s)
    % Call Vehicle Model
    vehicle;
    % define objective
    prob.minimize(-ay); % Maximum GG Envelope Radius
    prob.set_initial(delta,0);
    prob.set_initial(beta,0);
    prob.set_initial(Sxf,0);
    prob.set_initial(Sxr,0);
    prob.set_initial(dpsi,0);
    if j>2
        prob.set_initial(delta,GG.delta(j-1));
        prob.set_initial(beta,GG.beta(j-1));
        prob.set_initial(Sxf,GG.Sxf(j-1));
        prob.set_initial(Sxr,GG.Sxr(j-1));
        prob.set_initial(dpsi,GG.dpsi(j-1));
    end
    % define constraints
    prob.subject_to(Mz == 0);
    prob.subject_to(ax == ax_target);
    prob.subject_to(ay - V*dpsi == 0);
    prob.subject_to(-maxSa<=Saf<=maxSa);
    prob.subject_to(-maxSa<=Sar<=maxSa);
    % optimization results
    prob.solver('ipopt', p_opts, s_opts);
    % security catch in case failure
    try
        x = prob.solve();
        GG.ax(j) = x.value(ax);
        GG.ay(j) = x.value(ay);
        GG.delta(j) = x.value(delta);
        GG.beta(j) = x.value(beta);
        GG.dpsi(j) = x.value(dpsi);
        GG.Sxf(j) = x.value(Sxf);
        GG.Sxr(j) = x.value(Sxr);
        GG.Sar(j) = x.value(Sar);
        GG.Saf(j) = x.value(Saf);
    catch
        GG.ax(j) = NaN;
        GG.ay(j) = NaN;
        fprintf("Combined Slip Failed at V - %0.2f [m/s] & j - %0.2f [m/s^2] \n", V, j)
    end
end
GG.ax = [maxAx,GG.ax];
GG.ay = [0,GG.ay];


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



