
addpath('C:\Users\PC5\Documents\casadi-3.6.7-windows64-matlab2018b')
import casadi.*

% HoosierR25=mfeval.readTIR('Hoosier_18_75_10_R25B');

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
mass_front = 0.5095; % mass distribution (front heavy)
track_width = 1.21;  % track width
Inertia = 106;
v_min = 10; % [m/s] minimum speed for GG calculation
v_max = (max_rpm/FDR)*pi*2*R/60; % maximum speed
PMaxLimit = 80; % [kW] Power Limit
P = 9; % tire pressue (psi)
IA = 0; % inclination angle (rad)


% % Bounds for Path Constraints
maxDelta = del_max; % maximum steering angle (rad)
maxSa = deg2rad(10);
maxBeta = deg2rad(20);
maxSxfr = 0.1;
maxSxfl = 0.1;
maxSxrr = 0.1;
maxSxrl = 0.1;
maxDpsi = deg2rad(180); % deg/s to rad/s

% % IPOPT Settings
p_opts = struct;
s_opts = struct;
opts.expand =true;
p_opts.print_time = 0;
s_opts.print_level = 0; % 0: no display, 5: display


% % Mesh Discretization
Vnum = 30;        % number of speed variations
Gnum = 10;        % number of combine ax/ay variations
velocityRange = linspace(v_min,v_max-5, Vnum); % Discrete Velocity Points

tic
% % Create empty performance envelope GG
GG = struct();
GG.speed = struct();
figure


% % Steady State Speed Setting
for i = 1:numel(velocityRange)    
    % empty array for ay
    GG.speed(i).ay = zeros(1, Gnum+2);
    % current iterated speed
    V = velocityRange(i);
    GG.speed(i).speed = V;
    % Maximum Forward Acceleration
    prob = casadi.Opti();    
    % Initialise Decision Variables
    delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);       % steering angle (rad)
    beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);           % body slip (rad)
    Sxfl = prob.variable(); prob.subject_to(-maxSxfl<=Sxfl<=maxSxfl);           % front left slip ratio
    Sxfr = prob.variable(); prob.subject_to(-maxSxfr<=Sxfr<=maxSxfr);           % front right slip ratio
    Sxrl = prob.variable(); prob.subject_to(-maxSxrl<=Sxrl<=maxSxrl);           % rear left slip ratio
    Sxrr = prob.variable(); prob.subject_to(-maxSxrr<=Sxfr<=maxSxrr);           % rear right slip ratio
    dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);           % Yaw rate (rad/s)
    % Call Vehicle Model
    vehicle;
    % define initial guess
    prob.set_initial(delta,0);
    prob.set_initial(beta,0);
    prob.set_initial(Sxfl,0);
    prob.set_initial(Sxfr,0);
    prob.set_initial(Sxrl,0);
    prob.set_initial(Sxrr,0);
    prob.set_initial(dpsi,0);
    if i>1
        prob.set_initial(Sxfl,GG.speed(i-1).Sxfl(1));
        prob.set_initial(Sxfr,GG.speed(i-1).Sxfr(1));
        prob.set_initial(Sxfl,GG.speed(i-1).Sxrl(1));
        prob.set_initial(Sxfr,GG.speed(i-1).Sxrr(1));
    end
    % define constraints
    prob.subject_to(Mz == 0);
    prob.subject_to(ay - V*dpsi == 0);
    prob.solver('ipopt', p_opts, s_opts);

    % % acceleration G solver
    prob.minimize(-ax); 
    x = prob.solve(); 
    maxAx = x.value(ax);
    GG.speed(i).delta(1) = x.value(delta);
    GG.speed(i).beta(1) = x.value(beta);
    GG.speed(i).dpsi(1) = x.value(dpsi);
    % slip ratio
    GG.speed(i).Sxfl(1) = x.value(Sxfl);
    GG.speed(i).Sxfr(1) = x.value(Sxfr);
    GG.speed(i).Sxrl(1) = x.value(Sxrl);
    GG.speed(i).Sxrr(1) = x.value(Sxrr);
    % slip angle
    GG.speed(i).Safl(1) = x.value(Safl);
    GG.speed(i).Safr(1) = x.value(Safr);
    GG.speed(i).Sarl(1) = x.value(Sarl);
    GG.speed(i).Sarr(1) = x.value(Sarr);

    % % braking G solver
    % redefine initial guess
    if i>1
        prob.set_initial(Sxfl,GG.speed(i-1).Sxfl(1));
        prob.set_initial(Sxfr,GG.speed(i-1).Sxfr(1));
        prob.set_initial(Sxfl,GG.speed(i-1).Sxrl(1));
        prob.set_initial(Sxfr,GG.speed(i-1).Sxrr(1));
    end
    prob.minimize(ax);
    x = prob.solve();
    minAx = x.value(ax);
    GG.speed(i).delta(numel(velocityRange)+2) = x.value(delta);
    GG.speed(i).beta(numel(velocityRange)+2) = x.value(beta);
    GG.speed(i).dpsi(numel(velocityRange)+2) = x.value(dpsi);
    % slip ratio
    GG.speed(i).Sxfl(numel(velocityRange)+2) = x.value(Sxfl);
    GG.speed(i).Sxfr(numel(velocityRange)+2) = x.value(Sxfr);
    GG.speed(i).Sxrl(numel(velocityRange)+2) = x.value(Sxrl);
    GG.speed(i).Sxrr(numel(velocityRange)+2) = x.value(Sxrr);
    % slip angle
    GG.speed(i).Safl(numel(velocityRange)+2) = x.value(Safl);
    GG.speed(i).Safr(numel(velocityRange)+2) = x.value(Safr);
    GG.speed(i).Sarl(numel(velocityRange)+2) = x.value(Sarl);
    GG.speed(i).Sarr(numel(velocityRange)+2) = x.value(Sarr);


    % % First combine ax/ay solver
    prob = casadi.Opti();
    % Decision Variables
    delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);        % steering angle (rad)
    beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
    Sxfl = prob.variable(); prob.subject_to(-maxSxfl<=Sxfl<=maxSxfl);            % front left slip ratio
    Sxfr = prob.variable(); prob.subject_to(-maxSxfr<=Sxfr<=maxSxfr);            % front right slip ratio
    Sxrl = prob.variable(); prob.subject_to(-maxSxrl<=Sxrl<=maxSxrl);            % rear left slip ratio
    Sxrr = prob.variable(); prob.subject_to(-maxSxrr<=Sxfr<=maxSxrr);            % rear right slip ratio               % rear slip ratio
    dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);            % Yaw rate (rad/s)
    % Call Vehicle Model
    vehicle;
    % objective
    prob.minimize(-ay);
    % set initial guess
    prob.set_initial(delta,0);
    prob.set_initial(beta,0);
    prob.set_initial(Sxfl,0);
    prob.set_initial(Sxfr,0);
    prob.set_initial(Sxrl,0);
    prob.set_initial(Sxrr,0);
    prob.set_initial(dpsi,0);
    % define constraints
    prob.subject_to(Mz == 0);
    prob.subject_to(ax == maxAx);
    prob.subject_to(ay - V*dpsi == 0);
    prob.subject_to(-maxSa<=Safl<=maxSa);
    prob.subject_to(-maxSa<=Safr<=maxSa);
    prob.subject_to(-maxSa<=Sarl<=maxSa);
    prob.subject_to(-maxSa<=Sarr<=maxSa);
    % optimization results
    prob.solver('ipopt', p_opts, s_opts);
    % security catch in case failure
    try
        x = prob.solve();
        GG.speed(i).ax(2) = x.value(ax);
        GG.speed(i).ay(2) = x.value(ay);
        GG.speed(i).delta(2) = x.value(delta);
        GG.speed(i).beta(2) = x.value(beta);
        GG.speed(i).dpsi(2) = x.value(dpsi);
        % slip ratio
        GG.speed(i).Sxfl(2) = x.value(Sxfl);
        GG.speed(i).Sxfr(2) = x.value(Sxfr);
        GG.speed(i).Sxrl(2) = x.value(Sxrl);
        GG.speed(i).Sxrr(2) = x.value(Sxrr);
        % slip angle
        GG.speed(i).Safl(2) = x.value(Safl);
        GG.speed(i).Safr(2) = x.value(Safr);
        GG.speed(i).Sarl(2) = x.value(Sarl);
        GG.speed(i).Sarr(2) = x.value(Sarr);
    catch
        GG.speed(i).ax(2) = NaN;
        GG.speed(i).ay(2) = NaN;
        fprintf("Combined Slip Failed at V - %0.2f [m/s] \n", V)
    end

    % % equal spread ax to -ax
    GG.speed(i).ax = [maxAx,linspace(maxAx, minAx, Gnum), minAx];

    % Lateral G Solver
    for j = 3:numel(GG.speed(i).ax)-1
        ax_target = GG.speed(i).ax(j);
        prob = casadi.Opti();
        % Decision Variables
        delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);        % steering angle (rad)
        beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
        Sxfl = prob.variable(); prob.subject_to(-maxSxfl<=Sxfl<=maxSxfl);            % front left slip ratio
        Sxfr = prob.variable(); prob.subject_to(-maxSxfr<=Sxfr<=maxSxfr);            % front right slip ratio
        Sxrl = prob.variable(); prob.subject_to(-maxSxrl<=Sxrl<=maxSxrl);            % rear left slip ratio
        Sxrr = prob.variable(); prob.subject_to(-maxSxrr<=Sxfr<=maxSxrr);            % rear right slip ratio
        dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);            % Yaw rate (rad/s)
        % Call Vehicle Model
        vehicle;
        % define objective
        prob.minimize(-ay); % Maximum GG Envelope Radius
        % set initial guess
        prob.set_initial(delta,0);
        prob.set_initial(beta,0);
        prob.set_initial(Sxfl,0);
        prob.set_initial(Sxfr,0);
        prob.set_initial(Sxrl,0);
        prob.set_initial(Sxrr,0);
        prob.set_initial(dpsi,0);
        if j>3
            prob.set_initial(delta,GG.speed(i).delta(j-1));
            prob.set_initial(beta,GG.speed(i).beta(j-1));
            prob.set_initial(Sxfl,GG.speed(i).Sxfl(j-1));
            prob.set_initial(Sxfr,GG.speed(i).Sxfr(j-1));
            prob.set_initial(Sxrl,GG.speed(i).Sxrl(j-1));
            prob.set_initial(Sxrr,GG.speed(i).Sxrr(j-1));
            prob.set_initial(dpsi,GG.speed(i).dpsi(j-1));
        end
        % define constraints
        prob.subject_to(Mz == 0);
        prob.subject_to(ax == ax_target);
        prob.subject_to(ay - V*dpsi == 0);
        prob.subject_to(-maxSa<=Safl<=maxSa);
        prob.subject_to(-maxSa<=Safr<=maxSa);
        prob.subject_to(-maxSa<=Sarl<=maxSa);
        prob.subject_to(-maxSa<=Sarr<=maxSa);
        % optimization results
        prob.solver('ipopt', p_opts, s_opts);
        % security catch in case failure
        try
            x = prob.solve();
            GG.speed(i).ax(j) = x.value(ax);
            GG.speed(i).ay(j) = x.value(ay);
            GG.speed(i).delta(j) = x.value(delta);
            GG.speed(i).beta(j) = x.value(beta);
            GG.speed(i).dpsi(j) = x.value(dpsi);
            % slip ratio
            GG.speed(i).Sxfl(j) = x.value(Sxfl);
            GG.speed(i).Sxfr(j) = x.value(Sxfr);
            GG.speed(i).Sxrl(j) = x.value(Sxrl);
            GG.speed(i).Sxrr(j) = x.value(Sxrr);
            % slip angle
            GG.speed(i).Safl(j) = x.value(Safl);
            GG.speed(i).Safr(j) = x.value(Safr);
            GG.speed(i).Sarl(j) = x.value(Sarl);
            GG.speed(i).Sarr(j) = x.value(Sarr);
        catch
            GG.speed(i).ax(j) = NaN;
            GG.speed(i).ay(j) = NaN;
            fprintf("Combined Slip Failed at V - %0.2f [m/s] & j - %0.2f [m/s^2] \n", V, j)
        end
    end
    % store maximum ay at each speed
    GG.speed(i).aymax = max(GG.speed(i).ay);
    % 3D plot GG diagram
    z = GG.speed(i).speed* ones(size(GG.speed(i).ax));  
    plot3(GG.speed(i).ay, GG.speed(i).ax, z, 'LineWidth', 1.5)
    hold on
end

%% Smaller/Cleaner GGV Array

GGV = struct;

% Get Car Forward Velocity into an array that is inline with collapsed
% acceleration array
vCar = repmat([GG.speed.speed],Gnum+2,1);
vCar = vCar(:);

GGV.vCar = vCar';
GGV.gLong = [GG.speed.ax];
GGV.gLat = [GG.speed.ay];

%%
% % Performance Envelope for maximum cornering G
% Extract maximum performance at each speed
ymax = zeros(2,length(GG.speed));
for i = 1:length(GG.speed)
    ymax(1,i) = GG.speed(i).speed;
    ymax(2,i) = GG.speed(i).aymax;
end
% Create cornering G performance envelope
performance = struct();
Rnum = length(ymax);
performance.speed = zeros(1,Rnum);
performance.radius = zeros(1,Rnum);
for v = 1:length(ymax)
    speed = ymax(1,v);
    ay = ymax(2,v);
    radius = speed^2/ay;
    performance.speed(v) = speed;
    performance.radius(v) = radius;
end
% interpolate radius at each speed
PerfEnv =spline(performance.radius,performance.speed);


%%
% % Split performace envelope into Accel and Brake
% Empty 3xN array for [speed,ay,ax]
performance.v = zeros(1,Vnum*(Gnum+2));
performance.ax = zeros(1,Vnum*(Gnum+2));
performance.ay = zeros(1,Vnum*(Gnum+2));
for i = 1:Vnum
    for j = 1:Gnum+2
        % speed value
        performance.v((i-1)*(Gnum+2)+j) = GG.speed(i).speed;
        % ay value
        performance.ay((i-1)*(Gnum+2)+j) = GG.speed(i).ay(j);
        % ax value
        performance.ax((i-1)*(Gnum+2)+j) = GG.speed(i).ax(j);
    end
end
% split performance envelope in half
accel = struct(); brake = struct();
idxpos = performance.ax>=0;
idxneg = performance.ax<0;
accel.ax = performance.ax(idxpos);accel.ay = performance.ay(idxpos);accel.v = performance.v(idxpos);
brake.ax = performance.ax(idxneg);brake.ay = performance.ay(idxneg);brake.v = performance.v(idxneg);



%%

% % 3D interpolate Performance Envelope
% define max and min for interpolant
axmax = max(accel.ax);
axmin = min(brake.ax);
aymax = max([accel.ay(:);brake.ax(:)]);
aymin = min([accel.ay(:);brake.ay(:)]);
vmax = 35; vmin = 0;  
% deceleration
figure
plot3(brake.ay(:),brake.ax(:),brake.v(:),'.');
[vq,axq]=meshgrid(linspace(vmin, vmax, 100), linspace(axmin, 0, 100));
ayBrake =scatteredInterpolant(brake.v(:),brake.ax(:),brake.ay(:),'natural','boundary'); 
ayq = ayBrake(vq,axq);
hold on
surf(ayq,axq,vq);
xlabel('ay'); ylabel('ax'); zlabel('V');
title('Interpolated Surface for Brake Envelope');
colorbar;
% acceleration
figure
plot3(accel.ay(:),accel.ax(:),accel.v(:),'.');
[vq,axq]=meshgrid(linspace(vmin, vmax, 100), linspace(0, axmax, 100));
ayAccel =scatteredInterpolant(accel.v(:),accel.ax(:),accel.ay(:),'natural','boundary'); 
ayq = ayAccel(vq,axq);
hold on
surf(ayq,axq,vq);
xlabel('ay'); ylabel('ax'); zlabel('V');
title('Interpolated Surface for Acceleration Envelope');
colorbar;


%%

% interpolate performance envelope
axAccel =scatteredInterpolant(accel.v(:),accel.ay(:),accel.ax(:),'natural','boundary');
ayAccel =scatteredInterpolant(accel.v(:),accel.ax(:),accel.ay(:),'natural','boundary');
vAccel =scatteredInterpolant(accel.ax(:),accel.ay(:),accel.v(:),'natural','boundary');    
axBrake =scatteredInterpolant(brake.v(:),brake.ay(:),brake.ax(:),'natural','boundary');
ayBrake =scatteredInterpolant(brake.v(:),brake.ax(:),brake.ay(:),'natural','boundary');
vBrake =scatteredInterpolant(brake.ax(:),brake.ay(:),brake.v(:),'natural','boundary');    

toc
