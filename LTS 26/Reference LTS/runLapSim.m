function results = runLapSim(GGV, track, settings)

% Properties
track.sectorDistance = mean(diff(track.dist));
velocity_ub  = max(GGV.vCar); % Measured Limits - GGV
velocity_lb  = min(GGV.vCar); % Measured Limits - GGV

% Ingest GGV Data
u   =  GGV.vCar';
ax  =  GGV.gLong';
ay  =  GGV.gLat';

% Interpolation
idx = (ax > 0); 
PosAxInterp     = scatteredInterpolant(u(idx), ay(idx), ax(idx),'natural','nearest');
idx = (ax < 0); 
NegAxInterp     = scatteredInterpolant(u(idx), ay(idx), ax(idx),'natural','nearest');

%  Maximum Velocity as a function of curvature
LSP.velRange = unique(u);
for i = 1:numel(LSP.velRange)
    idx = find(u == LSP.velRange(i));
    kt = ay(idx)./(u(idx).^2);
    LSP.ktMax(i) = max(kt);
    LSP.vel(i) = LSP.velRange(i);
end

% Typical Corner Extremes - 1/4.5
KtRange = 0.0001:0.001:(1/3);
velInterp = interp1(LSP.ktMax, LSP.vel, KtRange,'linear','extrap');
velInterp = min(max(velInterp, 1),velocity_ub); % Lower bound capped to hard-capped to 1 m/s

% Limit Speed Calculation
LimitSpeedProfile = zeros(numel(track.dist),1);
for i = 1:numel(track.dist)
    
    % Steady State Cornering Interpolant - Limit Speed Envelope
    cornerVelInterp = interp1(KtRange,velInterp,abs(track.C2(i)),'spline','extrap');
    LimitSpeedProfile(i) = cornerVelInterp;
end

% Forward Acceleration Calculation
% Identify Apices
[val, locs] = findpeaks(-LimitSpeedProfile,"MinPeakDistance",6);

ForwardAccelerationProfile = zeros(numel(track.C2),1);
ForwardAccelerationProfile(locs(val == max(val))) = LimitSpeedProfile(locs(val == max(val)));

% Start at the Slowest Corner
for i = locs(val == max(val)):length(track.C2)-1

    curvature   = track.C2(i);
    currentVel  = ForwardAccelerationProfile(i);
    currentAy   = currentVel^2 * curvature;

    if currentVel < velocity_lb % Provide feasible ax for lower than pre-calculated speeds
        Ax = max(0,PosAxInterp(velocity_lb,currentAy));
    else % Continue with usual calculations
        Ax = max(0,PosAxInterp(currentVel,currentAy));
    end
    
    ForwardAccelerationProfile(i+1) = min(LimitSpeedProfile(i+1),abs(sqrt((currentVel^2) + 2*Ax*track.sectorDistance)));
end

if settings.track.bContinuousLap % Endurance
    ForwardAccelerationProfile(1) = ForwardAccelerationProfile(end); % Connect end of lap and start of lap
else % AutoX, Starting at 0 Velocity
    ForwardAccelerationProfile(1) = 0;
end

for i = 1: locs(val == max(val))
    
    curvature = track.C2(i);
    currentVel = ForwardAccelerationProfile(i);
    currentAy = currentVel^2 * abs(curvature);

    if currentVel < velocity_lb % Provide feasible ax for lower than pre-calculated speeds
        Ax = max(0,PosAxInterp(velocity_lb,currentAy));
    else % Continue with usual calculations
        Ax = max(0,PosAxInterp(currentVel,currentAy));
    end
    
    ForwardAccelerationProfile(i+1) = min(LimitSpeedProfile(i+1),abs(sqrt((currentVel^2) + 2*Ax*track.sectorDistance)));

end 

% Braking Speed Calculation
BrakingDecelerationProfile = zeros(length(track.C2),1);

BrakingDecelerationProfile(locs(end)) = LimitSpeedProfile(locs(end));

for i = locs(end):-1:2

    curvature = track.C2(i);
    currentVel = BrakingDecelerationProfile(i);
    currentAy = currentVel^2 * abs(curvature);

    if currentVel < velocity_lb % Provide feasible ax for lower than pre-calculated speeds
        Ax = min(0,NegAxInterp(velocity_lb,currentAy)); % Protection against surface extrapolation to negative velocities
    else % Continue with usual calculations
        Ax = min(0,NegAxInterp(currentVel,currentAy)); % Protection against surface extrapolation to negative velocities
    end
    
    BrakingDecelerationProfile(i-1) = min(LimitSpeedProfile(i-1),abs(sqrt((currentVel^2) - 2*Ax*track.sectorDistance)));

end

for i = locs(end):length(track.C2)
    BrakingDecelerationProfile(i) = LimitSpeedProfile(i); 
end 

% Calculate final states
% Velocity Profile
finalVel = min([LimitSpeedProfile';ForwardAccelerationProfile';BrakingDecelerationProfile'])';

% Longitudinal Acceleration
finalAx = zeros(length(track.C2),1);
for i = 1:length(track.C2)-1
    finalAx(i) = (finalVel(i+1)^2 - finalVel(i)^2)/(2*track.sectorDistance);
end
finalAx(length(finalVel)) = finalAx(1);

% Lateral Acceleration
finalAy = track.C2'.*finalVel.^2;

% Yaw Rate
yawRate = finalAy./finalVel;

% Time
dt = 1./finalVel;
t = cumtrapz(track.dist,dt);

figure(1); clf;
hold on
plot(track.dist, LimitSpeedProfile, 'r')
plot(track.dist, ForwardAccelerationProfile, 'g')
plot(track.dist, BrakingDecelerationProfile, 'b')
plot(track.dist, finalVel,'k','LineWidth',2)
legend('LimitSpeed','ForwardSpeed','BrakingSpeed','FinalVelocity')
title(['LapTime: ' num2str(t(end)) ' (s)'])

% Extract Results and map in sLap
results = struct;

results.settings = settings;
results.t        = t;
results.vCar     = finalVel;
results.ax       = finalAx;
results.ay       = finalAy;
results.dpsi     = yawRate;
results.sLap     = track.dist;
results.Curv     = track.C2;
results.track_position = track.pos;

end