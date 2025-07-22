% % Vehicle Model

% % Resolve lataral and longitudinal limit
prob = casadi.Opti();

% % define unknowns
delta = prob.variable(); prob.subject_to(-del_max<=delta<=del_max);          % steering angle (rad)
beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);            % yaw rate (rad/s)
Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);                % front slip ratio
Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);                % rear slip ratio

% % Equations of Motions
% Distance from CG to front axle and CG to rear
lf = wheelbase*mass_front;
lr = wheelbase*(1-mass_front);
% velocities in vehicle fixed coordinates
dx = V*cos(beta);
dy = V*sin(beta);
% front/rear slip angles (Milliken pg.148)
Saf = -delta + atan((dy + lf*dpsi)/dx);
Sar = atan((dy - lr*dpsi)/dx);
% aerodynamics
Drag = 0.5*den*(V^2)*CDs*farea;
% Powertrian limit
v_weak = 80/3.6; % field weakening start when speed = 80kmh for 333.75V setting % v_weak = 86.5/3.6; % 360V setting
if V < v_weak  %24.03 % 360V setting
    tractive = 0.8*Ipeak*220*FDR/R-Drag;
else    
    Iweak = ((220-0)/(v_weak-v_max))*V+220-((220-0)/(v_weak-v_max))*v_weak;
    tractive = 0.8*Ipeak*Iweak*FDR/R;
end
Gtractive = tractive/(mass*9.81);

% % Tire model
% tire parameters
A =1800; B =1.5; C =25; D =1; E =20;
% pure slip
Fxpf = A*sin(B*atan(C*Sxf));
Fypf = -A*sin(B*atan(C*tan(Saf)));
Fxpr = A*sin(B*atan(C*Sxr));
Fypr = -A*sin(B*atan(C*tan(Sar)));
% combined slip
Fxf = Fxpf * cos(D*atan(E*tan(Saf)));
Fyf = Fypf * cos(D*atan(E*Sxf));
Fxr = Fxpr * cos(D*atan(E*tan(Sar)));
Fyr = Fypr * cos(D*atan(E*Sxr));

% % Equations of Motions
% sum of forces in vehicle fixed coordinates
Fy = Fyf*cos(delta) + Fxf*sin(delta) + Fyr;
Fx = Fxf*cos(delta) - Fyf*sin(delta) + Fxr;
Mz = lf*(Fyf*cos(delta) + Fxf*sin(delta)) - lr*Fyr;
% accelerations in path tangential coordinates
ax = (1/mass * (Fy*sin(beta) + Fx*cos(beta)-Drag))/9.81;
ay = (1/mass * (Fy*cos(beta) - Fx*sin(beta)))/9.81;

