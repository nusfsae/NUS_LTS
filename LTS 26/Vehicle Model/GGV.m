%% generate full performance envelope under each speed

import casadi.*

% % Bounds for Path Constraints
maxp = 20;                           % maximum radius of GG diagram (m/s^2)
maxDelta = del_max;                  % maximum steering angle (rad)
maxSa = deg2rad(10);                 % maximum slip angle (deg)
maxBeta = deg2rad(20);               % maximum body slip (deg)
maxSxfr = 0.1;                       % maximum front right slip ratio (-)
maxSxfl = 0.1;                       % maximum front left slip ratio (-)
maxSxrr = 0.1;                       % maximum rear right slip ratio (-)
maxSxrl = 0.1;                       % maximum rear left slip ratio (-)
maxDpsi = deg2rad(180);              % maximum yaw rate (deg/s)

% % IPOPT Settings
opts = struct();
opts.print_time = false;
opts.ipopt.print_level = 1;
opts.ipopt.tol = 1e-6;
opts.ipopt.acceptable_tol = 1e-4;
opts.ipopt.acceptable_iter = 15;
opts.ipopt.max_iter = 3000;


% % Mesh Discretization
v_min = 10;                          % minimum speed for GG calculation (m/s)
v_max = (max_rpm/FDR)*pi*2*R/60;     % maximum speed (m/s)
Vnum = 30;                           % number of speed variations
Gnum = 20;                           % number of combine ax/ay variations
velocityRange = linspace(v_min,v_max-5, Vnum); % Discrete Velocity Points
failcount = 0;                       % number of failure in optimization

tic

% % Create empty performance envelope GG
GG = struct();
GG.speed = struct();

%%

% % Steady State Speed Setting
for i = 1:numel(velocityRange)
    % empty array for ay
    GG.speed(i).ay = zeros(1, Gnum);
    % current iterated speed
    V = velocityRange(i);
    GG.speed(i).speed = V;
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
            opti.set_initial(delta,GG.speed(i).delta(j-1));
            opti.set_initial(Sxfl,GG.speed(i).Sxfl(j-1));
            opti.set_initial(Sxfr,GG.speed(i).Sxfr(j-1));
            opti.set_initial(Sxrl,GG.speed(i).Sxrl(j-1));
            opti.set_initial(Sxrr,GG.speed(i).Sxrr(j-1));
            opti.set_initial(dpsi,GG.speed(i).dpsi(j-1));
        end
        % define constraints
        opti.subject_to(ax_res ==0);
        opti.subject_to(ay_res ==0);
        opti.subject_to(Mz == 0);
        opti.subject_to( ay-V*dpsi == 0);
        opti.subject_to(-maxSa<=Safl<=maxSa);
        opti.subject_to(-maxSa<=Safr<=maxSa);
        opti.subject_to(-maxSa<=Sarl<=maxSa);
        opti.subject_to(-maxSa<=Sarr<=maxSa);
        opti.subject_to(Fx<=Fxpwt);
        min_Fz = 0;  % Minimum normal force (N)
        opti.subject_to(Fzfl >= min_Fz);
        opti.subject_to(Fzfr >= min_Fz);
        opti.subject_to(Fzrl >= min_Fz);
        opti.subject_to(Fzrr >= min_Fz);
        % Speed-dependent initial guess
        if V > 20
            opti.set_initial(p, maxp*0.3);  % Conservative initial guess
        else
            opti.set_initial(p, maxp*0.9);  % Aggressive for low speed
        end
        % optimization results
        opti.solver('ipopt', opts);
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
            failcount = failcount+1;
            if j >1
                GG.speed(i).delta(j) = GG.speed(i).delta(j-1);
                GG.speed(i).beta(j) = GG.speed(i).beta(j-1);
                GG.speed(i).dpsi(j) = GG.speed(i).dpsi(j-1);
                % slip ratio
                GG.speed(i).Sxfl(j) = GG.speed(i).Sxfl(j-1);
                GG.speed(i).Sxfr(j) = GG.speed(i).Sxfr(j-1);
                GG.speed(i).Sxrl(j) = GG.speed(i).Sxrl(j-1);
                GG.speed(i).Sxrr(j) = GG.speed(i).Sxrr(j-1);
                % slip angle
                GG.speed(i).Safl(j) = GG.speed(i).Safl(j-1);
                GG.speed(i).Safr(j) = GG.speed(i).Safr(j-1);
                GG.speed(i).Sarl(j) = GG.speed(i).Sarl(j-1);
                GG.speed(i).Sarr(j) = GG.speed(i).Sarr(j-1);
                fprintf("Combined Slip Failed at V - %0.2f [m/s] & j - %0.2f [m/s^2] \n", V, j)
            else
                GG.speed(i).delta(j) = 0;
                GG.speed(i).beta(j) = 0;
                GG.speed(i).dpsi(j) = 0;
                % slip ratio
                GG.speed(i).Sxfl(j) = 0;
                GG.speed(i).Sxfr(j) = 0;
                GG.speed(i).Sxrl(j) = 0;
                GG.speed(i).Sxrr(j) = 0;
                % slip angle
                GG.speed(i).Safl(j) = 0;
                GG.speed(i).Safr(j) = 0;
                GG.speed(i).Sarl(j) = 0;
                GG.speed(i).Sarr(j) = 0;
                fprintf("Combined Slip Failed at V - %0.2f [m/s] & j - %0.2f [m/s^2] \n", V, j)
            end
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


%% 3D interpolate Performance Envelope
for i = 1:Vnum
    for j = 1:Gnum
        % speed value
        performance.v((i-1)*(Gnum)+j) = GG.speed(i).speed;
        % ay value
        performance.ay((i-1)*(Gnum)+j) = GG.speed(i).ay(j);
        % ax value
        performance.ax((i-1)*(Gnum)+j) = GG.speed(i).ax(j);
        % steering
        performance.delta((i-1)*(Gnum)+j) = GG.speed(i).delta(j);
        % slip ratio
        performance.Sxfl((i-1)*(Gnum)+j) = GG.speed(i).Sxfl(j);
        performance.Sxfr((i-1)*(Gnum)+j) = GG.speed(i).Sxfr(j);
        performance.Sxrl((i-1)*(Gnum)+j) = GG.speed(i).Sxrl(j);
        performance.Sxrr((i-1)*(Gnum)+j) = GG.speed(i).Sxrr(j);
        % slip angle
        performance.Safl((i-1)*(Gnum)+j) = GG.speed(i).Safl(j);
        performance.Safr((i-1)*(Gnum)+j) = GG.speed(i).Safr(j);
        performance.Sarl((i-1)*(Gnum)+j) = GG.speed(i).Sarl(j);
        performance.Sarr((i-1)*(Gnum)+j) = GG.speed(i).Sarr(j);
    end
end
% split performance envelope in accel and brake
accel = struct(); brake = struct();
idxpos = performance.ax>=0;
idxneg = performance.ax<0;
accel.ax = performance.ax(idxpos);accel.ay = performance.ay(idxpos);accel.v = performance.v(idxpos);
brake.ax = performance.ax(idxneg);brake.ay = performance.ay(idxneg);brake.v = performance.v(idxneg);

% delete entries with NaN speed value
idx = find(isnan(accel.v));
accel.ax(idx) =[]; accel.ay(idx) =[]; accel.v(idx) =[];
idx = find(isnan(brake.v));
brake.ax(idx) =[]; brake.ay(idx) =[]; brake.v(idx) =[];
idx = find(isnan(performance.ay));
performance.delta(idx) =[]; performance.ay(idx) =[];performance.v(idx) =[];performance.ax(idx) =[];
performance.Sxfl(idx) =[];performance.Sxfr(idx) =[];performance.Sxrl(idx) =[];performance.Sxrr(idx) =[];
performance.Safl(idx) =[];performance.Safr(idx) =[];performance.Sarl(idx) =[];performance.Sarr(idx) =[];

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
% steering
figure
plot3(performance.ay(:),rad2deg(performance.delta(:)),performance.v(:),'.');
[vq,ayq]=meshgrid(linspace(vmin, vmax, 100), linspace(0, aymax, 100));
findDelta =scatteredInterpolant(performance.ay(:),performance.v(:),performance.delta(:),'natural','boundary');
deltaq = rad2deg(findDelta(ayq,vq));
hold on
surf(ayq,deltaq,vq);
xlabel('ay'); ylabel('Delta'); zlabel('V');
title('Interpolated Surface for Steering Envelope');
colorbar;

%% slip ratio
% FL
figure
nexttile
temp = performance;
idxzero = find(abs(temp.Sxfl) <=0.0001);
temp.ax(idxzero) =[];temp.Sxfl(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ax(:),temp.Sxfl(:),temp.v(:),'.');
[axq,vq]=meshgrid(linspace(axmin, axmax, 100),linspace(vmin, vmax, 100));
findSxfl =scatteredInterpolant(temp.ax(:),temp.v(:),temp.Sxfl(:),'natural','boundary');
Sxflq = findSxfl(axq,vq);
hold on
surf(axq,Sxflq,vq);
xlabel('ax'); ylabel('Sx'); zlabel('V');
title('FL Slip Ratio');
% FR
nexttile
temp = performance;
idxzero = find(abs(temp.Sxfr) <=0.0001);
temp.ax(idxzero) =[];temp.Sxfr(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ax(:),temp.Sxfr(:),temp.v(:),'.');
[axq,vq]=meshgrid(linspace(axmin, axmax, 100),linspace(vmin, vmax, 100));
findSxfr =scatteredInterpolant(temp.ax(:),temp.v(:),temp.Sxfr(:),'natural','boundary');
Sxfrq = findSxfr(axq,vq);
hold on
surf(axq,Sxfrq,vq);
xlabel('ax'); ylabel('Sx'); zlabel('V');
title('FR Slip Ratio');
% RL
nexttile
temp = performance;
idxzero = find(abs(temp.Sxrl) <=0.0001);
temp.ax(idxzero) =[];temp.Sxrl(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ax(:),temp.Sxrl(:),temp.v(:),'.');
[axq,vq]=meshgrid(linspace(axmin, axmax, 100),linspace(vmin, vmax, 100));
findSxrl =scatteredInterpolant(temp.ax(:),temp.v(:),temp.Sxrl(:),'natural','boundary');
Sxrrq = findSxrl(axq,vq);
hold on
surf(axq,Sxrrq,vq);
xlabel('ax'); ylabel('Sx'); zlabel('V');
title('RL Slip Ratio');
% RR
nexttile
temp = performance;
idxzero = find(abs(temp.Sxrr) <=0.0001);
temp.ax(idxzero) =[];temp.Sxrr(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ax(:),temp.Sxrr(:),temp.v(:),'.');
[axq,vq]=meshgrid(linspace(axmin, axmax, 100),linspace(vmin, vmax, 100));
findSxrr =scatteredInterpolant(temp.ax(:),temp.v(:),temp.Sxrr(:),'natural','boundary');
Sxrrq = findSxrr(axq,vq);
hold on
surf(axq,Sxrrq,vq);
xlabel('ax'); ylabel('Sx'); zlabel('V');
title('RR Slip Ratio');

%% slip angle
% FL
figure
nexttile
temp = performance;
idxzero = find(abs(temp.Safl) <=0.0001);
temp.ay(idxzero) =[];temp.Safl(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ay(:),temp.Safl(:),temp.v(:),'.');
[ayq,vq]=meshgrid(linspace(0, aymax, 100),linspace(vmin, vmax, 100));
findSafl =scatteredInterpolant(temp.ay(:),temp.v(:),temp.Safl(:),'natural','boundary');
Saflq = findSafl(ayq,vq);
hold on
surf(ayq,Saflq,vq);
xlabel('ay'); ylabel('Sa'); zlabel('V');
title('FL Slip Angle');
% FR
nexttile
temp = performance;
idxzero = find(abs(temp.Safr) <=0.0001);
temp.ay(idxzero) =[];temp.Safr(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ay(:),temp.Safr(:),temp.v(:),'.');
[ayq,vq]=meshgrid(linspace(aymin, aymax, 100),linspace(vmin, vmax, 100));
findSafr =scatteredInterpolant(temp.ay(:),temp.v(:),temp.Safr(:),'natural','boundary');
Safrq = findSafr(ayq,vq);
hold on
surf(ayq,Safrq,vq);
xlabel('ay'); ylabel('Sa'); zlabel('V');
title('FR Slip Angle');
% RL
nexttile
temp = performance;
idxzero = find(abs(temp.Sarl) <=0.0001);
temp.ay(idxzero) =[];temp.Sarl(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ay(:),temp.Sarl(:),temp.v(:),'.');
[ayq,vq]=meshgrid(linspace(aymin, aymax, 100),linspace(vmin, vmax, 100));
findSarl =scatteredInterpolant(temp.ay(:),temp.v(:),temp.Sarl(:),'natural','boundary');
Sarrq = findSarl(ayq,vq);
hold on
surf(ayq,Sarrq,vq);
xlabel('ay'); ylabel('Sa'); zlabel('V');
title('RL Slip Angle');
% RR
nexttile
temp = performance;
idxzero = find(abs(temp.Sarr) <=0.0001);
temp.ay(idxzero) =[];temp.Sarr(idxzero) =[];temp.v(idxzero) =[];
plot3(temp.ay(:),temp.Sarr(:),temp.v(:),'.');
[ayq,vq]=meshgrid(linspace(aymin, aymax, 100),linspace(vmin, vmax, 100));
findSarr =scatteredInterpolant(temp.ay(:),temp.v(:),temp.Sarr(:),'natural','boundary');
Sarrq = findSarr(ayq,vq);
hold on
surf(ayq,Sarrq,vq);
xlabel('ay'); ylabel('Sa'); zlabel('V');
title('RR Slip Angle');
%%

% interpolate performance envelope
axAccel =scatteredInterpolant(accel.v(:),accel.ay(:),accel.ax(:),'natural','boundary');
ayAccel =scatteredInterpolant(accel.v(:),accel.ax(:),accel.ay(:),'natural','boundary');
vAccel =scatteredInterpolant(accel.ax(:),accel.ay(:),accel.v(:),'natural','boundary');
axBrake =scatteredInterpolant(brake.v(:),brake.ay(:),brake.ax(:),'natural','boundary');
ayBrake =scatteredInterpolant(brake.v(:),brake.ax(:),brake.ay(:),'natural','boundary');
vBrake =scatteredInterpolant(brake.ax(:),brake.ay(:),brake.v(:),'natural','boundary');

toc
