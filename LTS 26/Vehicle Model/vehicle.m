% % Vehicle Model

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

% Power Output
PowerOut = Fx*V/1000; % [kW]

% accelerations in path tangential coordinates
ax = (1/mass * (Fy*sin(beta) + Fx*cos(beta) - Drag));
ay = (1/mass * (Fy*cos(beta) - Fx*sin(beta)));

