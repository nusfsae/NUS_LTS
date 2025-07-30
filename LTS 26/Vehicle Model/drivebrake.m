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
    % calculate accelerated speed
    v = sqrt(v0^2+2*ax*distance);
    if v<=sim.speed(i+1)
        sim.speed(i+1)=v;
    end
end

hold on; plot(dist,sim.speed);
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
    v = sqrt(v0^2-2*ax*distance);
    % replace speed value if below boundary
    if v<=sim.speed(i-1)
        sim.speed(i-1)=v;
    end
end

% store ax/ay
for i = 1:length(C2)-1
    % turn radius of current point
    radius = 1/C2(i);
    % calculate kinematics
    v = sim.speed(i);
    ay = v^2/radius;
    sim.ay(i) =ay;
    % different drive/brake
    ax = axBrake(v,ay);
    sim.ax(i) =ax;
    if sim.speed(i+1)>sim.speed(i)
        ax = axAccel(v,ay);
        sim.ax(i) =ax;
    end
end
sim.ax(length(C2)) =sim.ax(length(C2)-1);
sim.ay(length(C2)) =sim.ay(length(C2)-1);