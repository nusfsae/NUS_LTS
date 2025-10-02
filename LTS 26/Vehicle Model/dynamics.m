% % evaluate dynamic performance 
% corner
for i = 1:length(C2)  
    radius = abs(1/C2(i));
    % if radius larger than 30m, assume straight
    if radius > 70      
        sim.speed(i,1) = v_max;
        sim.latG(i,1) = 0;   
    % cornering scenario
    else 
        vy = ppval(PerfEnv,radius);
        sim.speed(i,1) = vy;  
    end
end
% check rolling start
if static == true
    sim.speed(1) = 0;
end
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
% brake
for i = length(C2):-1:2
    % turn radius of current point
    radius = abs(1/C2(i));
    % distance to the next point
    distance = dist(i)-dist(i-1);
    % current speed and lateral acceleration
    v0 = sim.speed(i);
    ay = v0^2/radius;
    % available longitudinal deceleraton
    ax = axBrake(v0,ay);
    v = sqrt(v0^2-2*ax*distance);  % take note ax is a negative value
    % replace speed value if below boundary
    if v<=sim.speed(i-1)
        sim.speed(i-1)=v;
    end
end
% % store ax/ay
% calculate ay
sim.ay = C2'.*sim.speed.^2;
% calculate ax
for i = 1:length(C2)-1
    v0 = sim.speed(i);
    v1 = sim.speed(i+1);
    sim.ax(i) = (v1^2-v0^2)/(2*(dist(i+1)-dist(i)));
end
sim.ax = transpose(sim.ax);
% acceleration at end point
if static == true
    sim.ax(end+1) = axAccel(sim.speed(end),sim.ay(end));
else 
    sim.ax(end+1) = (sim.speed(1)^2-sim.speed(end)^2)/(2*dist(end));
end

