% % evaluate acceleration performance and modify from BSP

% accelerate
for i = 1:length(C2)-1
    % turn radius of current point
    radius = abs(1/C2(i));
    % distance to the next point
    distance = dist(i+1)-dist(i);
    % current speed and lateral acceleration
    v0 = sim.speed(i);
    ay = v0^2/radius;
    % available longitudinal acceleraton
    ax = axAccel(v0,ay);
    v = sqrt(v0^2+2*ax*distance);
    if v<=sim.speed(i+1)
        sim.speed(i+1)=v;
    end
end

% brake
for i = 2:length(C2)
    % turn radius of current point
    radius = abs(1/C2(i));
    % distance to the next point
    distance = dist(i)-dist(i-1);
    % current speed and lateral acceleration
    v0 = sim.speed(i);
    ay = v0^2/radius;
    % available longitudinal deceleraton
    ax = axBrake(v0,ay);
    v = sqrt(v0^2+2*ax*distance);
    % store ax/ay values
    sim.ax(i-1) = ax;
    sim.ay(i-1) = ay;
    if v<=sim.speed(i-1)
        sim.speed(i-1)=v;
        sim.ay(i-1)=v^2/radius;
    end
end