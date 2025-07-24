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
del_max = deg2rad(25); % 0.565;  % maximum steering angle (rad)
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


% Bounds for Path Constraints
maxSa = deg2rad(10);
maxBeta = deg2rad(10);
maxSxf = 0.2;
maxSxr = 0.2;

% IPOPT Settings
p_opts = struct;
s_opts = struct;
p_opts.print_time = 0;
s_opts.print_level = 0;

% Mesh Discretization
Vnum = 10;        % number of speed variations
Gnum = 10;        % number of longG variations
velocityRange = linspace(v_min,v_max, Vnum); % Discrete Velocity Points
alpha_range = linspace(-pi/2, pi/2, Gnum); % Discrete GG orientation Points

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
    GG.speed(i).p   = zeros(1,Gnum);
end


% % Steady State Speed Setting
for i = 1:numel(velocityRange)
    V = velocityRange(i);
    GG.speed(i).speed = V;

    % Solve the first point externally
    % use this as initial guess for the subsequent point

    alpha = alpha_range(1);
    prob = casadi.Opti();
    
    % % define unknowns
    delta = prob.variable(); prob.subject_to(0<=delta<=del_max);          % steering angle (rad)
    beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
    Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);                % front slip ratio
    Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);                % rear slip ratio
    p   = prob.variable(); prob.subject_to(0<=p<=3);   % GG Envelope Radius

    % Set Targets
    ax_target = p * sin(alpha - beta) * 9.81;
    ay_target = p * cos(alpha - beta) * 9.81;
    dpsi = ay_target/V;

    vehicle;

    % define objective
    prob.minimize(-p^2); % Maximum GG Envelope Radius

    % define initial guess
    prob.set_initial(p,0.1);
    prob.set_initial(delta,0);
    prob.set_initial(beta,0);
    prob.set_initial(Sxf,0);
    prob.set_initial(Sxr,0);

    % define constraints
    prob.subject_to(PowerOut<=80);
    prob.subject_to(Mz == 0);
    prob.subject_to(-maxSa<=Saf<=maxSa);
    prob.subject_to(-maxSa<=Sar<=maxSa);
    prob.subject_to(ax == ax_target);
    prob.subject_to(ay == ay_target);

    % optimization results
    prob.solver('ipopt', p_opts, s_opts);
    x = prob.solve();
    GG.speed(i).p(1)  = x.value(p);
    GG.speed(i).ax(1) = x.value(ax);
    GG.speed(i).ay(1) = x.value(ay);
    GG.speed(i).delta(1) = x.value(delta);
    GG.speed(i).beta(1) = x.value(beta);
    GG.speed(i).dpsi(1) = x.value(dpsi);
    GG.speed(i).Sxf(1) = x.value(Sxf);
    GG.speed(i).Sxr(1) = x.value(Sxr);
    GG.speed(i).Sar(1) = x.value(Sar);
    GG.speed(i).Saf(1) = x.value(Saf);

    for j = 2:numel(alpha_range)
        alpha = alpha_range(j);

        % Solver for Envelope in One Move
        % Create optimisation problem
        prob = casadi.Opti();
        
        % % define unknowns
        delta = prob.variable(); prob.subject_to(-del_max<=delta<=del_max);          % steering angle (rad)
        beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
        Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);                % front slip ratio
        Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);                % rear slip ratio
        p   = prob.variable(); prob.subject_to(0<=p<=3);   % GG Envelope Radius
    
        % Set Targets
        ax_target = p * sin(alpha - beta) * 9.81;
        ay_target = p * cos(alpha - beta) * 9.81;
        dpsi = ay_target/V;
    
        vehicle;
    
        % define objective
        prob.minimize(-p^2); % Maximum GG Envelope Radius
    
        % define initial guess
%         prob.set_initial(p,GG.speed(i).p(j-1));
%         prob.set_initial(delta,GG.speed(i).delta(j-1));
%         prob.set_initial(beta,GG.speed(i).beta(j-1));
%         prob.set_initial(Sxf,GG.speed(i).Sxf(j-1));
%         prob.set_initial(Sxr,GG.speed(i).Sxr(j-1));

        prob.set_initial(p,0);
        prob.set_initial(delta,0);
        prob.set_initial(beta,0);
        prob.set_initial(Sxf,0);
        prob.set_initial(Sxr,0);
    
        % define constraints
        prob.subject_to(PowerOut<=80);
        prob.subject_to(Mz == 0);
        prob.subject_to(-maxSa<=Saf<=maxSa);
        prob.subject_to(-maxSa<=Sar<=maxSa);
        prob.subject_to(ax == ax_target);
        prob.subject_to(ay == ay_target);

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
            fprintf("Combined Slip Failed at V - %0.2f [m/s] & Alpha - %0.2f [deg] \n", V, rad2deg(alpha))
        
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

