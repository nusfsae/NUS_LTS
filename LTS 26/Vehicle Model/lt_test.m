% others
V = 15;
IA =0;
para =H1675;
mass = 262;
den = 1.196;                         % air density (kgm^-3)
farea = 1.157757;                    % frontel area (m^2)
CLc = 3.828;                         % CL cornering
CDc = 1.403;                         % CD cornering
ab_c = 0.528;                        % aero balance cornering (front)
CLs = 4.034;                         % CL straight line
CDs = 1.543;                         % CD straight line
mass = 262;                          % vehicle mass (kg)
d = 1.21;                        % track width (m)
cg_f = 0.5095;                       % mass bias to front (-)
wheelbase = 1.531;                   % wheelbase (m)
cg_h = 0.256;                        % CG height (m)
ab_s = 0.549; 
% aero
Drag = 0.5*den*(V^2)*CDc*farea;
Lift = 0.5*den*(V^2)*CLc*farea;
AeroF = Lift*ab_c;
AeroR = Lift*(1-ab_c);
% no load transfer
Sxfl = 0.000;
Sxfr = 0.000 ;
Sxrl = -0.003 ;
Sxrr = -0.008 ;
Safl = -0.132 ;
Safr = -0.130 ;
Sarl = -0.141 ;
Sarr = -0.128 ;
% weight per wheel wo LT
Fz = (mass*9.81)/4;
Fzfl = Fz+AeroF/2;
Fzfr = Fz+AeroF/2;
Fzrl = Fz+AeroR/2;
Fzrr = Fz+AeroR/2;

% % ay w lt = 17.797
% ay_lt = 17.797;
% Fysum_lt = mass*ay_lt;

[Fyfr,Fxfr] = MF52(Sxfr,Safr,Fzfr,IA,para);
[Fyfl,Fxfl] = MF52(Sxfl,Safl,Fzfl,IA,para);
[Fyrl,Fxrl] = MF52(Sxrl,Sarl,Fzrl,IA,para);
[Fyrr,Fxrr] = MF52(Sxrr,Sarr,Fzrr,IA,para);

Fsum = Fyfr+Fyfl+Fyrl+Fyrr;

disp(Fsum);

% LT
ay = 17.797;
ax =0;

% load transfer
latLT = Fz*(ay/9.81)*cg_h/d;
longLT = Fz*(ax/9.81)*cg_h/wheelbase;
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
Fsum_lt = Fyfr+Fyfl+Fyrl+Fyrr;
disp(Fysum_lt);
disp(Fsum-Fysum_lt)