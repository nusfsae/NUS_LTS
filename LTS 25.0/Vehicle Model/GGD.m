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


import casadi.*


tic
% % Create empty performance envelope GG
GG = struct();
% Vnum = 400;                     % number of speed variations
Gnum = 50;                     % number of longG variations
% GG.speed = struct();
% for i = 1:Vnum
GG.ax = zeros(1,Gnum);
GG.ay = zeros(1,Gnum);
GG.delta = zeros(1,Gnum);
GG.beta = zeros(1,Gnum);
GG.dpsi = zeros(1,Gnum);
GG.Sxf = zeros(1,Gnum);
GG.Sxr = zeros(1,Gnum);
GG.Saf = zeros(1,Gnum);
GG.Sar = zeros(1,Gnum);
% end

% % Create required cornering G envelope
% require = struct();
% curvnum = 300;                   % 0.1:0.1:30 turn radius (m)
% latnum = 300;                    % 0.1:0.01:3 lateral acceleration (G)
% require.curv = struct();
% for i = 1:latnum
%     require.curv(i).speed = zeros(1,latnum);
%     require.curv(i).lateral = zeros(1,latnum);
% end
% for i = 1:curvnum
%     curvature = i/10;
%     for lateral = 0:0.01:latnum/0.01
%         speed = sqrt(lateral*curvature/9.81);
%         require.curv(i).speed = speed;
%         require.curv(i).lateral = lateral;
%     end
% end


% % Steady State Speed Setting
% for i = 10:40
    % V = i; % (m/s)  
V = 7; % (m/s)


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
prob.subject_to(ax<=Gtractive);
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
prob.subject_to(ay-V*dpsi/9.81 == 0);
prob.subject_to(-deg2rad(10)<=Saf<=deg2rad(10));  
prob.subject_to(-deg2rad(10)<=Sar<=deg2rad(10)); 
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
    % % estimate initial guess
    % if i<Gnum/2
    %     guessDelta = deg2rad(0);
    %     guessBeta = deg2rad(0);
    %     guessDpsi = 0;
    %     guessSxf = 0*i;
    %     guessSxr = 0*i;
    % else
    %     guessDelta = deg2rad(00);
    %     guessBeta = deg2rad(0);
    %     guessDpsi = 0;
    %     guessSxf = 0*i;
    %     guessSxr = 0*i;
    % end
    % define initial guess 
    prob.set_initial(delta,deg2rad(0));
    prob.set_initial(beta,deg2rad(0));
    prob.set_initial(dpsi,0);
    prob.set_initial(Sxf,0);
    prob.set_initial(Sxr,0);
    % define constraints
    prob.subject_to(ax == GG.ax(i));
    prob.subject_to(Mz == 0);
    prob.subject_to(ay-V*dpsi/9.81 == 0);
    prob.subject_to(-deg2rad(10)<=Saf<=deg2rad(10));  
    prob.subject_to(-deg2rad(10)<=Sar<=deg2rad(10)); 
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


%%
figure
plot(GG.ay,GG.ax)

%%
figure
yyaxis left
plot(rad2deg(GG.delta))
yyaxis right
plot(rad2deg(GG.beta))
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