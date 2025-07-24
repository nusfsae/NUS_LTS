clear; clc;

% % Add Folder Dependencies to Path
import casadi.*

% % Vehicle Model
mass = 262;  % vehicle mass (kg)
den = 1.196;  % air density
farea = 1.157757;  % frontel area (m^2)
CLc = 3.782684;
CDc = 1.410518;
CLs = 4.116061;
CDs = 1.54709;  
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

% Bounds for Path Constraints
maxDelta = deg2rad(25); % maximum steering angle (rad)
maxSa = deg2rad(10);
maxBeta = deg2rad(10);
maxSxf = 0.1;
maxSxr = 0.1;
maxDpsi = deg2rad(120); % deg/s to rad/s

% IPOPT Settings
p_opts = struct;
s_opts = struct;
p_opts.print_time = 0;
s_opts.print_level = 0;

% Mesh Discretization
Vnum = 10;        % number of speed variations
Gnum = 10;        % number of longG variations
velocityRange = linspace(v_min,v_max - 5, Vnum); % Discrete Velocity Points

tic
% % Create empty performance envelope GG
GG = struct();
GG.speed = struct();
for i = 1:Vnum
    GG.speed(i).ax = zeros(1,Gnum);
    GG.speed(i).ay = zeros(1,Gnum);
    GG.speed(i).delta = zeros(1,Gnum);
    GG.speed(i).beta = zeros(1,Gnum);
    GG.speed(i).dpsi = zeros(1,Gnum);
    GG.speed(i).Sxf = zeros(1,Gnum);
    GG.speed(i).Sxr = zeros(1,Gnum);
    GG.speed(i).Saf = zeros(1,Gnum);
    GG.speed(i).Sar = zeros(1,Gnum);
end


% % Steady State Speed Setting
for i = 1:numel(velocityRange)
    V = velocityRange(i);
    GG.speed(i).speed = V;

    % Maximum Forward Acceleration
    prob = casadi.Opti();
    
    % Initialise Decision Variables
    delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);          % steering angle (rad)
    beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);     % body slip (rad)
    Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);         % front slip ratio
    Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);         % rear slip ratio
    dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);   % Yaw rate (rad/s)

    % Call Vehicle Model
    vehicle;

    % define initial guess
    prob.set_initial(delta,0);
    prob.set_initial(beta,0);
    prob.set_initial(Sxf,0);
    prob.set_initial(Sxr,0);
    prob.set_initial(dpsi,0);

    % define constraints
    prob.subject_to(PowerOut<=PMaxLimit); % Power Limits
    prob.subject_to(Mz == 0);
    prob.subject_to(ay - V*dpsi == 0);

    prob.solver('ipopt', p_opts, s_opts);

    % Maximum Acceleration
    prob.minimize(-ax); % Objective
    x = prob.solve(); % optimization results
    maxAx = x.value(ax);

    % Maximum Deceleration
    prob.minimize(ax); % Objective
    x = prob.solve(); % Optimisation Results
    minAx = x.value(ax);

    GG.speed(i).ax = [maxAx, linspace(maxAx, minAx, Gnum), minAx]; % Pad the array with zeros
    GG.speed(i).ay = zeros(1, numel(GG.speed(i).ax));

    for j = 2:numel(GG.speed(i).ax)-1
        
        ax_target = GG.speed(i).ax(j);
        prob = casadi.Opti();
        
        % Decision Variables
        delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);          % steering angle (rad)
        beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
        Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);                % front slip ratio
        Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);                % rear slip ratio
        dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);             % Yaw rate (rad/s)

        % Call Vehicle Model   
        vehicle;
    
        % define objective
        prob.minimize(-ay); % Maximum GG Envelope Radius

        prob.set_initial(delta,0);
        prob.set_initial(beta,0);
        prob.set_initial(Sxf,0);
        prob.set_initial(Sxr,0);
        prob.set_initial(dpsi,0);
    
        % define constraints
        prob.subject_to(Mz == 0);
        prob.subject_to(ax == ax_target);
        prob.subject_to(ay - V*dpsi == 0);
        prob.subject_to(-maxSa<=Saf<=maxSa);
        prob.subject_to(-maxSa<=Sar<=maxSa);

        % optimization results
        prob.solver('ipopt', p_opts, s_opts);

        try
        
            x = prob.solve();
            GG.speed(i).ax(j) = x.value(ax);
            GG.speed(i).ay(j) = x.value(ay);
            GG.speed(i).delta(j) = x.value(delta);
            GG.speed(i).beta(j) = x.value(beta);
            GG.speed(i).dpsi(j) = x.value(dpsi);
            GG.speed(i).Sxf(j) = x.value(Sxf);
            GG.speed(i).Sxr(j) = x.value(Sxr);
            GG.speed(i).Sar(j) = x.value(Sar);
            GG.speed(i).Saf(j) = x.value(Saf);

        catch
         
            GG.speed(i).ax(j) = NaN;
            GG.speed(i).ay(j) = NaN;
            fprintf("Combined Slip Failed at V - %0.2f [m/s] & Ax - %0.2f [m/s^2] \n", V, ax_target)
        
        end
        
    end

    % store maximum ay at each speed
    GG.speed(i).aymax = max(GG.speed(i).ay);

    % 3D plot GG diagram
    z = i * ones(size(GG.speed(i).ax));  
    plot3(GG.speed(i).ay, GG.speed(i).ax, z, 'LineWidth', 1.5)
    hold on
end


% % % Extract maximum performance at each speed
% ymax = zeros(2,length(GG.speed));
% for i = 1:length(GG.speed)
%     ymax(1,i) = GG.speed(i).speed;
%     ymax(2,i) = GG.speed(i).aymax;
% end
% 
% 
% % % Create cornering G performance envelope
% performance = struct();
% Rnum = 20;
% performance.speed = zeros(1,Rnum);
% performance.radius = zeros(1,Rnum);
% for v = 1:length(ymax)
%     speed = ymax(1,v);
%     ay = ymax(2,v);
%     radius = speed^2/(ay);
%     performance.speed(v) = speed;
%     performance.radius(v) = radius;
% end
% % interpolate radius at each speed
% PerfEnv = spline(performance.speed,performance.radius);

