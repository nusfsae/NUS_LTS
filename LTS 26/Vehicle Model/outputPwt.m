function pwt = outputPwt()
% Powertrain Settings
pwt.max_rpm   = 5500;    % maximum wheel speed (rpm)
pwt.FDR       = 3.36;    % final drive ratio (-)
pwt.Ipeak     = 1;      % power percentage (-)
pwt.PMaxLimit = 80000;     % power limit (kW)

% Add a torque limit for throttle mapping
pwt.Tmax      = 200;    % max motor torque (Nm) (example)
end
