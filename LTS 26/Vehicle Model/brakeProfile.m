% % complete speed profile with braking

% return 2d array, local minima speed vs corresponded indices
local_min = localfinder(sim.speed);
count = length(local_min);

%for every interval between 2 local minima
for point = 1:count-1 
    % index number of local minima, with respect to the whole track map
    i1 = local_min(point,2);
    i2 = local_min(point+1,2);
    gap = i2-i1;   
    for i = 0:local_min(point,2) %for every point in the interval
        % ACTUAL index of iteration(start from the last point of interval)
        target = local_min(point+1,2)-i; 
        radius = abs(1/C2(target-1));              
        % differentiate straight line and corner aero properties
        if radius >30
            CL = CLs; 
            CD = CDs;
        else
            CL = CLc;
            CD = CDc;
        end
        % define distance and target speed
        distance = dist(target)-dist(target-1);
        v1 = sim.speed(target-1,1);
        v2 = sim.speed(target,1);              
        % Calculate velocity after braking with these conditions    
        calculateBrakeV;                   
        sim.speed(target-1,1) = braked_v;                 
    end        
end




