% % evaluate acceleration performance and modify from BSP

for i = 1:length(C2)-1
    % turn radius of current point
    radius = abs(1/C2(i));
    % distance to the next point
    distance = dist(i+1)-dist(i);
    % current speed and lateral acceleration
    v0 = sim.speed(i);
    ay = v0^2/radius;
    % available longitudinal acceleraton
    ax = findax(v0,ay);
    v = sqrt(v0^2+2*ax*distance);
    if v<=sim.speed(i+1)
        sim.speed(i+1)=v;
    end
end
