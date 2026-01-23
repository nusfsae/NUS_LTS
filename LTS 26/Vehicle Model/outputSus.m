function suspension= outputSus()
% Suspension Settings
suspension.del_max   = 0.565;   % maximum steering angle (rad)
suspension.R         = 0.2032;  % wheel radius (m)
suspension.P         = 9;       % tire pressure (psi)
suspension.IA        = 0;       % inclination angle (rad)  (note: you later treat IA as deg in MF52)
suspension.brakebias = 0.67;    % brake bias (-)
end