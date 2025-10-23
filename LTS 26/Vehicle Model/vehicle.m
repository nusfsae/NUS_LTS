% % Vehicle Model

% % Equations of Motions
% Load pre-computed ackerman coefficients
load('ackerman_coeffs.mat', 'p_inner', 'p_outer');
% CG location and vehicle dimensions
a = wheelbase*cg_f;
b = wheelbase-a;
d = track;
% steering angle with ackerman
[delta_in, delta_out] = ackerman(delta, [], p_inner, p_outer);
% velocities in vehicle fixed coordinates
dx = V*cos(beta);
dy = V*sin(beta);
% slip angles
Safr = -delta_in + atan((dy+a*dpsi)/(dx+d*dpsi/2));
Safl = -delta_out + atan((dy+a*dpsi)/(dx-d*dpsi/2));
Sarr = atan((dy-b*dpsi)/(dx+d*dpsi/2));
Sarl = atan((dy-b*dpsi)/(dx-d*dpsi/2));
% aerodynamics
Drag = 0.5*den*(V^2)*CDs*farea;
Lift = 0.5*den*(V^2)*CLs*farea;
AeroF = Lift*ab;
AeroR = Lift*(1-ab);
% normal load by mass
Fz = (mass*9.81)/4;
% load transfer
latLT = Fz*(ay_in/9.81)*cg_h/d;
longLT = Fz*(ax_in/9.81)*cg_h/wheelbase;
% wheel loads
Fzfl = Fz+AeroF/2-latLT-longLT;
Fzfr = Fz+AeroF/2+latLT-longLT;
Fzrl = Fz+AeroR/2-latLT+longLT;
Fzrr = Fz+AeroR/2+latLT-longLT;
% tire forces
[Fyfr,Fxfr] = MF52(Sxfr,Safr,Fzfr,IA,para);
[Fyfl,Fxfl] = MF52(Sxfl,Safl,Fzfl,IA,para);
[Fyrl,Fxrl] = MF52(Sxrl,Sarl,Fzrl,IA,para);
[Fyrr,Fxrr] = MF52(Sxrr,Sarr,Fzrr,IA,para);

% % Equations of Motions
% sum of forces in vehicle fixed coordinates
Fy = Fyfr*cos(delta_in)+Fyfl*cos(delta_out)+Fxfr*sin(delta_in)+Fxfl*sin(delta_out)+Fyrl+Fyrr;
Fx = Fxfr*cos(delta_in)+Fxfl*cos(delta_out)-Fyfr*sin(delta_in)+Fyfl*sin(delta_out)+Fxrl+Fxrr;
Mz = (a*(Fxfr*sin(delta_in)+Fxfl*sin(delta_out))+a*(Fyfr*cos(delta_in)+Fyfl*cos(delta_out))-b*(Fyrl+Fyrr)+d*(Fxfr*cos(delta_in)-Fxfl*cos(delta_out))/2+d*(Fxrr-Fxrl)/2+d*(Fyfl*sin(delta_out)-Fyfr*sin(delta_in))/2);

% % Powertrain model
Fxpwt = 0.9*Ipeak*220*FDR/R;
v_weak =86.5/3.6;
Iweak = ((220-0)/(v_weak-v_max))*V+220-((220-0)/(v_weak-v_max))*v_weak; 
if V>v_weak
    Fxpwt =0.9*Ipeak*Iweak*FDR/R;
end

% accelerations in path tangential coordinates
ax = (1/mass * (Fy*sin(beta) + Fx*cos(beta) - Drag));
ay = (1/mass * (Fy*cos(beta) - Fx*sin(beta)));

% residual control
ax_res = ax-ax_in;
ay_res = ay-ay_in;
