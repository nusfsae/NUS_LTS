
addpath('C:\Users\PC5\Documents\casadi-3.6.7-windows64-matlab2018b')
addpath(genpath(cd))
import casadi.*

% Chassis Settings
mass = 262;                          % vehicle mass (kg)
track = 1.21;                        % track width (m)
cg_f = 0.5095;                       % mass distribution (front heavy)
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
maxp = 30;                           % maximum radius of GG diagram (m/s^2)
maxDelta = del_max;                  % maximum steering angle (rad)
maxSa = deg2rad(10);                 % maximum slip angle (deg)
maxBeta = deg2rad(20);               % maximum body slip (deg)
maxSxfr = 0.1;                       % maximum front right slip ratio (-)
maxSxfl = 0.1;                       % maximum front left slip ratio (-)
maxSxrr = 0.1;                       % maximum rear right slip ratio (-)
maxSxrl = 0.1;                       % maximum rear left slip ratio (-)
maxDpsi = deg2rad(180);              % maximum yaw rate (deg/s)

% % IPOPT Settings
p_opts = struct;
s_opts = struct;
p_opts.print_time = 0;
s_opts.print_level = 0; % 0: no display, 5: display

% % Mesh Discretization
v_min = 10;       % minimum speed for GG calculation (m/s)
Vnum = 30;        % number of speed variations
Gnum = 30;        % number of combine ax/ay variations
velocityRange = linspace(v_min,v_max-5, Vnum); % Discrete Velocity Points

tic
% % Create empty performance envelope GG
GG = struct();
GG.speed = struct();

%%
figure

% % Steady State Speed Setting
for i = 1:numel(velocityRange)    
    % empty array for ay
    GG.speed(i).ay = zeros(1, Gnum);
    % current iterated speed
    V = velocityRange(i);
    GG.speed(i).speed = V;
    % Range of ax/ay combinations
    AngleRange = linspace(-pi/2,pi/2,Gnum);
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
        Sxfl = opti.variable(); opti.subject_to(-maxSxfl<=Sxfl<=maxSxfl);           % front left slip ratio
        Sxfr = opti.variable(); opti.subject_to(-maxSxfr<=Sxfr<=maxSxfr);           % front right slip ratio
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
        % define constraints
        opti.subject_to(ax_res ==0);
        opti.subject_to(ay_res ==0);
        opti.subject_to(Mz == 0);
        opti.subject_to(ay - V*dpsi == 0);
        opti.solver('ipopt', p_opts, s_opts);
        opti.subject_to(-maxSa<=Safl<=maxSa);
        opti.subject_to(-maxSa<=Safr<=maxSa);
        opti.subject_to(-maxSa<=Sarl<=maxSa);
        opti.subject_to(-maxSa<=Sarr<=maxSa);
        % optimization results
        opti.solver('ipopt', p_opts, s_opts);
        % objective
        opti.minimize(-p);
        % results
        try
            x = opti.solve();
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

% Empty 3xN array for [speed,ay,ax]
performance.v = zeros(1,Vnum*(Gnum));
performance.ax = zeros(1,Vnum*(Gnum));
performance.ay = zeros(1,Vnum*(Gnum));
for i = 1:Vnum
    for j = 1:Gnum
        % speed value
        performance.v((i-1)*(Gnum)+j) = GG.speed(i).speed;
        % ay value
        performance.ay((i-1)*(Gnum)+j) = GG.speed(i).ay(j);
        % ax value
        performance.ax((i-1)*(Gnum)+j) = GG.speed(i).ax(j);
    end
end
% split performance envelope in accel and brake
accel = struct(); brake = struct();
idxpos = performance.ax>=0;
idxneg = performance.ax<0;
accel.ax = performance.ax(idxpos);accel.ay = performance.ay(idxpos);accel.v = performance.v(idxpos);
brake.ax = performance.ax(idxneg);brake.ay = performance.ay(idxneg);brake.v = performance.v(idxneg);

%%
% delete entries with NaN speed value
idx = find(isnan(accel.v));
accel.ax(idx) =[]; accel.ay(idx) =[]; accel.v(idx) =[]; 
idx = find(isnan(brake.v));
brake.ax(idx) =[]; brake.ay(idx) =[]; brake.v(idx) =[];
%%

% % 3D interpolate Performance Envelope
% define max and min for interpolant
axmax = max(accel.ax);
axmin = min(brake.ax);
aymax = max([accel.ay(:);brake.ax(:)]);
aymin = min([accel.ay(:);brake.ay(:)]);
vmax = v_max; vmin = 0;  
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
