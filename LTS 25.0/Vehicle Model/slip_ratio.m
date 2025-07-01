function k = slip_ratio(slipAngle, angleChange)

%derivation pacejka, Tires and Vehicle Dynamics, pg 69

slipSpeedRatio = tan(angleChange);
contactSpeedRatio = tan(slipAngle);
k = slipSpeedRatio * contactSpeedRatio;

% add slip circle?

