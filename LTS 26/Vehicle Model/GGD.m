
import casadi.*

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
p_opts.print_time = 0;
s_opts.print_level = 0;

% % Mesh Discretization
Vnum = 10;        % number of speed variations
Gnum = 10;        % number of combine ax/ay variations
velocityRange = linspace(v_min,v_max - 5, Vnum); % Discrete Velocity Points

tic
% % Create empty performance envelope GG
GG = struct();
GG.speed = struct();
% for i = 1:Vnum
%     GG.speed(i).ax = zeros(1,Gnum);
%     GG.speed(i).ay = zeros(1,Gnum);
%     GG.speed(i).delta = zeros(1,Gnum);
%     GG.speed(i).beta = zeros(1,Gnum);
%     GG.speed(i).dpsi = zeros(1,Gnum);
%     GG.speed(i).Sxf = zeros(1,Gnum);
%     GG.speed(i).Sxr = zeros(1,Gnum);
%     GG.speed(i).Saf = zeros(1,Gnum);
%     GG.speed(i).Sar = zeros(1,Gnum);
%     GG.speed(i).V = zeros(1,Gnum);
% end


% % Steady State Speed Setting
for i = 1:numel(velocityRange)
    V = velocityRange(i);
    GG.speed(i).speed = V;
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
    % define constraints
    prob.subject_to(PowerOut<=PMaxLimit);
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
    GG.speed(i).Sxf(1) = x.value(Sxf);
    GG.speed(i).Sxr(1) = x.value(Sxr);
    GG.speed(i).Sar(1) = x.value(Sar);
    GG.speed(i).Saf(1) = x.value(Saf);

    % % braking G solver
    prob.minimize(ax);
    x = prob.solve();
    minAx = x.value(ax);
    GG.speed(i).delta(numel(velocityRange)+2) = x.value(delta);
    GG.speed(i).beta(numel(velocityRange)+2) = x.value(beta);
    GG.speed(i).dpsi(numel(velocityRange)+2) = x.value(dpsi);
    GG.speed(i).Sxf(numel(velocityRange)+2) = x.value(Sxf);
    GG.speed(i).Sxr(numel(velocityRange)+2) = x.value(Sxr);
    GG.speed(i).Sar(numel(velocityRange)+2) = x.value(Sar);
    GG.speed(i).Saf(numel(velocityRange)+2) = x.value(Saf);

    % % equal spread ax to -ax
    GG.speed(i).ax = [maxAx, linspace(maxAx, minAx, Gnum), minAx];
    GG.speed(i).ay = zeros(1, numel(GG.speed(i).ax));

    % Lateral G Solver
    for j = 2:numel(GG.speed(i).ax)-1
        ax_target = GG.speed(i).ax(j);
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
    z = GG.speed(i).speed* ones(size(GG.speed(i).ax));  
    plot3(GG.speed(i).ay, GG.speed(i).ax, z, 'LineWidth', 1.5)
    hold on
end

%%
% % Extract maximum performance at each speed
ymax = zeros(2,length(GG.speed));
for i = 1:length(GG.speed)
    ymax(1,i) = GG.speed(i).speed;
    ymax(2,i) = GG.speed(i).aymax;
end


% % Create cornering G performance envelope
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
PerfEnv =spline(performance.speed,performance.radius);
% 3xN array for [speed,ay,ax]
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

figure
plot3(accel.ay(:),accel.ax(:),accel.v(:));
testint =scatteredInterpolant(accel.ax(:),accel.ay(:),accel.v(:),'natural','boundary');
[xq, yq] = meshgrid(linspace(0,20), linspace(0,20));
zq = testint(xq, yq);  % interpolated values on the grid
figure;
surf(xq, yq, zq);
% shading interp;  % smooth surface
xlabel('ax'); ylabel('ay'); zlabel('V');
title('Interpolated Surface using scatteredInterpolant');
colorbar;

% interpolate performance envelope
% findax =scatteredInterpolant(performance.v(:),performance.ay(:),performance.ax(:),'natural','boundary');
% finday =scatteredInterpolant(performance.v(:),performance.ax(:),performance.ay(:),'natural','boundary');
% findv =scatteredInterpolant(performance.ax(:),performance.ay(:),performance.v(:),'natural','boundary');    

% 
% [xq, yq] = meshgrid(linspace(-20,20), linspace(0,20));
% zq = findv(xq, yq);  % interpolated values on the grid
% 
% % 4. Plot the surface
% figure;
% surf(xq, yq, zq);
% % shading interp;  % smooth surface
% xlabel('ax'); ylabel('ay'); zlabel('V');
% title('Interpolated Surface using scatteredInterpolant');
% colorbar;

% 
% figure
% plot(GG.speed(i).ay,GG.speed(i).ax)
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






% % % Vehicle Model
% 
% % % Resolve lataral and longitudinal limit
% function [ax,ay,Fx,Fy,Mz] = vehicle(x,mass,wheelbase,mass_front,V)
% 
% % define unknowns
% delta = x(1);  % steering angle
% beta = x(2);   % body slip
% dpsi = x(3);   % yaw rate
% Sxf = x(4);    % front slip ratio
% Sxr = x(5);    % rear slip ratio
% 
% % % Equations of Motions
% % Distance from CG to front axle and CG to rear
% lf = wheelbase*mass_front;
% lr = wheelbase*(1-mass_front);
% % velocities in vehicle fixed coordinates
% dx = V*cos(beta);
% dy = V*sin(beta);
% % front/rear slip angles (Milliken pg.148)
% Saf = -delta + atan((dy + lf*dpsi)/dx);
% Sar = atan((dy - lr*dpsi)/dx);
% 
% % % Tire model
% % tire parameters
% A =1800; B =1.5; C =25; D =1; E =20;
% % pure slip
% Fxpf = A*sin(B*atan(C*Sxf));
% Fypf = -A*sin(B*atan(C*tan(Saf)));
% Fxpr = A*sin(B*atan(C*Sxr));
% Fypr = -A*sin(B*atan(C*tan(Sar)));
% % combined slip
% Fxf = Fxpf * cos(D*atan(E*tan(Saf)));
% Fyf = Fypf * cos(D*atan(E*Sxf));
% Fxr = Fxpr * cos(D*atan(E*tan(Sar)));
% Fyr = Fypr * cos(D*atan(E*Sxr));
% 
% % % Equations of Motions
% % sum of forces in vehicle fixed coordinates
% Fy = Fyf*cos(delta) + Fxf*sin(delta) + Fyr;
% Fx = Fxf*cos(delta) - Fyf*sin(delta) + Fxr;
% Mz = lf*(Fyf*cos(delta) + Fxf*sin(delta)) - lr*Fyr;
% % accelerations in path tangential coordinates
% ax = -(1/mass * (Fy*sin(beta) + Fx*cos(beta)))/9.81;
% ay = -(1/mass * (Fy*cos(beta) - Fx*sin(beta)))/9.81;
% end




% % FMINCON Version for Braking G Solver
% problem.options = optimoptions('fmincon','Display','off','Algorithm','sqp');
% problem.objective = @(x) longitudinal(x,mass,wheelbase,mass_front,V);
% problem.x0 = [0 0 0 -0.2 -0.2]; % initial guess
% problem.lb = [-del_max -deg2rad(20) -deg2rad(90) -0.2 -0.2]; % lower bound
% problem.ub = [del_max deg2rad(20) deg2rad(90) 0.2 0.2]; % upper bound
% problem.solver = 'fmincon';
% [x, fval] = fmincon(problem);
% ax = -fval; % (g)
% ay = 0; % (g)
% GG.ax(1,num) = ax;
% GG.ay(1,num) = ay;

% % OPTIMPROBLEM Version for Lateral G Solver
% y = optimvar('y',5,"LowerBound",[-del_max -deg2rad(20) -deg2rad(90) -0.2 -0.2],"UpperBound",[del_max deg2rad(20) deg2rad(90) 0.2 0.2]);
% [ax,ay,Fx,Fy,Mz] = fcn2optimexpr(@vehicle,y,mass,wheelbase,mass_front,V);
% corner = optimproblem('Objective',ay,'ObjectiveSense','max');
% corner.Constraints.long = ax == GG.ax(1,index);
% corner.Constraints.yawMoment = Mz == 0;
% corner.Constraints.yawRate = ay-V*y(3) == 0;
% x0.y = [guessDelta guessBeta guessDpsi guessSxf guessSxr];
% x0.y = [0 0 0 0 0]; % initial guess
% opts = optimoptions('fmincon', 'Display', 'iter-detailed');
% [y, fval] = solve(corner, x0, 'Options', opts);
% ay = fval; % (g)
% GG.ay(1,index) = ay;
% GG.delta(1,index) = y.y(1);
% GG.beta(1,index) = y.y(2);
% GG.dpsi(1,index) = y.y(3);
% GG.Sxf(1,index) = y.y(4);
% GG.Sxr(1,index) = y.y(5);