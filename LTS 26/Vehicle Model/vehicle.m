% % Vehicle Model

% % Equations of Motions
% epsilon for continuity
eps = 1e-6;
% CG location and vehicle dimensions
a = wheelbase*mass_front;
b = wheelbase-a;
d = track_width;
% velocities in vehicle fixed coordinates
dx = V*cos(beta);
dy = V*sin(beta);
% slip angles 4-wheel model
% Reference --Vehicle dynamics and tire models: An overview
Safr = -delta + atan((dy+a*dpsi)/(dx+d*dpsi/2 + eps));
Safl = -delta + atan((dy+a*dpsi)/(dx-d*dpsi/2 + eps));
Sarr = atan((dy-b*dpsi)/(dx+d*dpsi/2 + eps));
Sarl = atan((dy-b*dpsi)/(dx-d*dpsi/2 + eps));
% aerodynamics
Drag = 0.5*den*(V^2)*CDs*farea;
Lift = 0.5*den*(V^2)*CLs*farea;
% normal load per tire
Fz = (mass*9.81+Lift)/4;

% % Tire model
% tire parameters
A =1800; B =1.5; C =25; D =1; E =20;
% front right
Fxpfr = A*sin(B*atan(C*Sxfr));
Fypfr = -A*sin(B*atan(C*tan(Safr)));
Fxfr = Fxpfr * cos(D*atan(E*tan(Safr)));
Fyfr = Fypfr * cos(D*atan(E*Sxfr));
% front left
Fxpfl = A*sin(B*atan(C*Sxfl));
Fypfl = -A*sin(B*atan(C*tan(Safl)));
Fxfl = Fxpfl * cos(D*atan(E*tan(Safl)));
Fyfl = Fypfl * cos(D*atan(E*Sxfl));
% rear right
Fxprr = A*sin(B*atan(C*Sxrr));
Fyprr = -A*sin(B*atan(C*tan(Sarr)));
Fxrr = Fxprr * cos(D*atan(E*tan(Sarr)));
Fyrr = Fyprr * cos(D*atan(E*Sxrr));
% rear left
Fxprl = A*sin(B*atan(C*Sxrl));
Fyprl = -A*sin(B*atan(C*tan(Sarl)));
Fxrl = Fxprl * cos(D*atan(E*tan(Sarl)));
Fyrl = Fyprl * cos(D*atan(E*Sxrl));


% % Equations of Motions
% sum of forces in vehicle fixed coordinates
Fy = (Fyfr+Fyfl)*cos(delta)+(Fxfr+Fxfl)*sin(delta)+Fyrl+Fyrr;
Fx = (Fxfr+Fxfl)*cos(delta)-(Fyfr+Fyfl)*sin(delta)+Fxrl+Fxrr;
Mz = (a*(Fxfr+Fxfl)*sin(delta)+a*(Fyfr+Fyfl)*cos(delta)-b*(Fyrl+Fyrr)+d*(Fxfr-Fxfl)*cos(delta)/2+d*(Fxrr-Fxrl)/2+d*(Fyfl-Fyfr)*sin(delta)/2);

% % Powertrain model
Fxpwt = 0.9*Ipeak*220*FDR/R;
v_weak =86.5/3.6;
Iweak = ((220-0)/(v_weak-v_max))*V+220-((220-0)/(v_weak-v_max))*v_weak; 
if V>v_weak
    Fxpwt =0.9*Ipeak*Iweak*FDR/R;
end
alpha =10;
Fx = smoothmin(Fxpwt,Fx,alpha);

% accelerations in path tangential coordinates
ax = (1/mass * (Fy*sin(beta) + Fx*cos(beta) - Drag));
ay = (1/mass * (Fy*cos(beta) - Fx*sin(beta)));

% resultant
p = sqrt(ax^2+ay^2);