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
GG = struct();
Vnum = floor(v_max-10)-10;        % number of speed variations
Gnum = 50;                        % number of longG variations
Vstart = 10;
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
for i = 1:floor(v_max)-Vstart
    V = i+9;
    GG.speed(i).speed = V;
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
    if V >= 20 && V <= 30
        maxDpsi = deg2rad(120);
    elseif V>30
        maxDpsi = deg2rad(140);
    else
        maxDpsi = deg2rad(90);
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
    GG.speed(i).ax(Gnum) = x.value(ax);
    GG.speed(i).ay(Gnum) = x.value(ay);
    GG.speed(i).delta(Gnum) = x.value(delta);
    GG.speed(i).beta(Gnum) = x.value(beta);
    GG.speed(i).dpsi(Gnum) = x.value(dpsi);
    GG.speed(i).Sxf(Gnum) = x.value(Sxf);
    GG.speed(i).Sxr(Gnum) = x.value(Sxr);
    GG.speed(i).Sar(Gnum) = x.value(Sar);
    GG.speed(i).Saf(Gnum) = x.value(Saf);


    % Acceleration G Solver
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
    prob.subject_to(ax<=Gtractive);
    % optimization results
    prob.solver('ipopt');
    x = prob.solve();
    GG.speed(i).ax(1) = x.value(ax);
    GG.speed(i).ay(1) = x.value(ay);
    GG.speed(i).delta(1) = x.value(delta);
    GG.speed(i).beta(1) = x.value(beta);
    GG.speed(i).dpsi(1) = x.value(dpsi);
    GG.speed(i).Sxf(1) = x.value(Sxf);
    GG.speed(i).Sxr(1) = x.value(Sxr);
    GG.speed(i).Sar(1) = x.value(Sar);
    GG.speed(i).Saf(1) = x.value(Saf);


    % Spread equally longitudinal G values across performance envelope
    GG.speed(i).ax = linspace(GG.speed(i).ax(1),GG.speed(i).ax(Gnum),length(GG.speed(i).ax));


    % Power Margin Lateral G Solver
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
    prob.subject_to(ax == GG.speed(i).ax(1));
    prob.subject_to(Mz == 0);
    prob.subject_to(ay-V*dpsi/9.81 == 0);
    prob.subject_to(-maxSa<=Saf<=maxSa);
    prob.subject_to(-maxSa<=Sar<=maxSa);
    % optimization results
    prob.solver('ipopt');
    x = prob.solve();
    GG.speed(i).ax(2) = x.value(ax);
    GG.speed(i).ay(2) = x.value(ay);
    GG.speed(i).delta(2) = x.value(delta);
    GG.speed(i).beta(2) = x.value(beta);
    GG.speed(i).dpsi(2) = x.value(dpsi);
    GG.speed(i).Sxf(2) = x.value(Sxf);
    GG.speed(i).Sxr(2) = x.value(Sxr);
    GG.speed(i).Sar(2) = x.value(Sar);
    GG.speed(i).Saf(2) = x.value(Saf);


    % Lateral G Solver
    for lat = 3:Gnum-1
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
        prob.subject_to(ax == GG.speed(i).ax(lat));
        prob.subject_to(Mz == 0);
        prob.subject_to(ay-V*dpsi/9.81 == 0);
        prob.subject_to(-maxSa<=Saf<=maxSa);
        prob.subject_to(-maxSa<=Sar<=maxSa);
        % optimization results
        prob.solver('ipopt');
        y = prob.solve();
        GG.speed(i).ax(lat) = y.value(ax);
        GG.speed(i).ay(lat) = y.value(ay);
        GG.speed(i).delta(lat) = y.value(delta);
        GG.speed(i).beta(lat) = y.value(beta);
        GG.speed(i).dpsi(lat) = y.value(dpsi);
        GG.speed(i).Sxf(lat) = y.value(Sxf);
        GG.speed(i).Sxr(lat) = y.value(Sxr);
        GG.speed(i).Sar(lat) = y.value(Sar);
        GG.speed(i).Saf(lat) = y.value(Saf);
    end

    % store maximum ay at each speed
    GG.speed(i).aymax = max(GG.speed(i).ay);

    % 3D plot GG diagram
    z = i * ones(size(GG.speed(i).ax));  
    plot3(GG.speed(i).ay, GG.speed(i).ax, z, 'LineWidth', 1.5)
    hold on
end


% % Extract maximum performance at each speed
ymax = zeros(2,length(GG.speed));
for i = 1:length(GG.speed)
    ymax(1,i) = GG.speed(i).speed;
    ymax(2,i) = GG.speed(i).aymax;
end


% % Create cornering G performance envelope
performance = struct();
Rnum = 20;
performance.speed = zeros(1,Rnum);
performance.radius = zeros(1,Rnum);
corner = casadi.Opti();
for radius = 1:Rnum
    % define variables
    speed = corner.variable(); corner.subject_to(0<=speed<=v_max);
    % require lateral acceleration (G)
    areq = speed^2/radius/9.81;
    % available lateral acceleration (G)
    ay = interp1(ymax(1,:),ymax(2,:),speed);
    % constraints
    corner.subject_to(ay>=areq);
    % objective
    corner.minimize(-speed);
    % initial guess
    corner.set_initial(speed=1);
    % solve cornering speed
    vy = corner.solver('ipopt');
    performance.speed(radius) = vy.value(speed);
    performance.radius(radius) = radius;
end


    


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