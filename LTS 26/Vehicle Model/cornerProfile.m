% Generate Boundary Speed Profile
% Evaluate maximum corner speed at each location

for i = 1:length(C2)  
    radius = abs(1/C2(i));
    % if radius larger than 30m, assume straight
    if abs(radius) > 30      
        sim.speed(i,1) = v_max;
        sim.latG(i,1) = 0;        
    else %cornering scenario
        vy = ppval(PerfEnv,radius);
        sim.speed(i,1) = vy;  
    end
end
