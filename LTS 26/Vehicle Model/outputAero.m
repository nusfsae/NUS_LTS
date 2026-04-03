function aero = outputAero()
% Aerodynamics Settings
aero.den   = 1.196;      % air density (kg/m^3)
aero.farea = 1.157757;  % frontal area (m^2)
aero.CLc   = 3.828;     % CL cornering
aero.CDc   = 1.403;     % CD cornering
aero.ab_c  = 0.528;     % aero balance cornering (front)
aero.CLs   = 4.034;     % CL straight line
aero.CDs   = 1.543;     % CD straight line
aero.ab_s  = 0.549;     % aero balance straight line (front)
end
