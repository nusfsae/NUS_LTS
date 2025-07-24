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
v_max = (max_rpm/FDR)*pi*2*R/60; % maximum speed


import casadi.*


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
V = 24; % (m/s)

% define variable constraints
% relax slip angle at high speed
if V >= 17 && V <= 23
    maxSa = deg2rad(8);
elseif V > 23 && V <= 25 || V <= 14
    maxSa = deg2rad(10);
elseif V > 25 && V <= 30 
    maxSa = deg2rad(12);
elseif V > 30 && V <= 33
    maxSa = deg2rad(20);
elseif V>33
    maxSa = deg2rad(22);
else
    maxSa = deg2rad(5);
end
maxBeta = deg2rad(20);
% relax yaw rate at high speed
% if V >= 20 && V <= 30
%     maxDpsi = deg2rad(120);
if V>30
    maxDpsi = deg2rad(140);
else
    maxDpsi = deg2rad(90);
end
if V == 20 
    maxDpsi = deg2rad(120);
end
if 21<=V<= 22
    maxDpsi = deg2rad(100);
end
maxSxf = 0.1;
maxSxr = 0.1;


% % Braking G Solver
prob = [];
vehicle;
% define objective
prob.minimize(ax);
% define initial guess
prob.set_initial(delta,0);
prob.set_initial(beta,0);
prob.set_initial(dpsi,0);
prob.set_initial(Sxf,-0.05);
prob.set_initial(Sxr,-0.05);
% optimization results
p_opts = struct;
s_opts = struct;
s_opts.print_level = 5;
prob.solver('ipopt', p_opts, s_opts);
x = prob.solve();
GG.ax(Gnum) = x.value(ax);
GG.ay(Gnum) = x.value(ay);
GG.delta(Gnum) = x.value(delta);
GG.beta(Gnum) = x.value(beta);
GG.dpsi(Gnum) = x.value(dpsi);
GG.Sxf(Gnum) = x.value(Sxf);
GG.Sxr(Gnum) = x.value(Sxr);
GG.Sar(Gnum) = x.value(Sar);
GG.Saf(Gnum) = x.value(Saf);


% % Acceleration G Solver
prob = [];
vehicle;
% define objective
prob.minimize(-ax);
% define initial guess
prob.set_initial(delta,0);
prob.set_initial(beta,0);
prob.set_initial(dpsi,0);
prob.set_initial(Sxf,0);
prob.set_initial(Sxr,0);
% define constraints
prob.subject_to(ax<=atractive);
% optimization results
prob.solver('ipopt');
x = prob.solve();
GG.ax(1) = x.value(ax);
GG.ay(1) = x.value(ay);
GG.delta(1) = x.value(delta);
GG.beta(1) = x.value(beta);
GG.dpsi(1) = x.value(dpsi);
GG.Sxf(1) = x.value(Sxf);
GG.Sxr(1) = x.value(Sxr);
GG.Sar(1) = x.value(Sar);
GG.Saf(1) = x.value(Saf);


% % Spread equally longitudinal G values across performance envelope
GG.ax = linspace(GG.ax(1),GG.ax(Gnum),length(GG.ax));


% % Power Margin Lateral G Solver
prob = [];
vehicle;
% define objective
prob.minimize(-ay);
% define initial guess
prob.set_initial(delta,deg2rad(0));
prob.set_initial(beta,deg2rad(0));
prob.set_initial(dpsi,0);
prob.set_initial(Sxf,0);
prob.set_initial(Sxr,0);
% define constraints
prob.subject_to(ax == GG.ax(1));
prob.subject_to(Mz == 0);
prob.subject_to(ay-V*dpsi == 0);
prob.subject_to(-maxSa<=Saf<=maxSa);
prob.subject_to(-maxSa<=Sar<=maxSa);
% optimization results
prob.solver('ipopt');
x = prob.solve();
GG.ax(2) = x.value(ax);
GG.ay(2) = x.value(ay);
GG.delta(2) = x.value(delta);
GG.beta(2) = x.value(beta);
GG.dpsi(2) = x.value(dpsi);
GG.Sxf(2) = x.value(Sxf);
GG.Sxr(2) = x.value(Sxr);
GG.Sar(2) = x.value(Sar);
GG.Saf(2) = x.value(Saf);


% % Lateral G Solver
for i = 3:Gnum-1
    prob = [];
    vehicle;
    % define objective
    prob.minimize(-ay);
    % define initial guess
    prob.set_initial(delta,deg2rad(0));
    prob.set_initial(beta,deg2rad(0));
    prob.set_initial(dpsi,0);
    prob.set_initial(Sxf,0);
    prob.set_initial(Sxr,0);
    % define constraints
    prob.subject_to(ax == GG.ax(i));
    prob.subject_to(Mz == 0);
    prob.subject_to(ay-V*dpsi == 0);
    prob.subject_to(-maxSa<=Saf<=maxSa);
    prob.subject_to(-maxSa<=Sar<=maxSa);
    % optimization results
    prob.solver('ipopt');
    y = prob.solve();
    GG.ax(i) = y.value(ax);
    GG.ay(i) = y.value(ay);
    GG.delta(i) = y.value(delta);
    GG.beta(i) = y.value(beta);
    GG.dpsi(i) = y.value(dpsi);
    GG.Sxf(i) = y.value(Sxf);
    GG.Sxr(i) = y.value(Sxr);
    GG.Sar(i) = y.value(Sar);
    GG.Saf(i) = y.value(Saf);
end
% end


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



