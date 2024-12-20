function final_lsp = brakeProfile(C2,dist,camber,tyre_model,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,Accel_LSP,tc_long,sen_long,phit,P,useMode)

final_lsp = Accel_LSP;
local_min = localfinder(Accel_LSP);%return 2d array, local minima speed vs corresponded indices
count = length(local_min);
slip_ang = slip_angle(C2);

for point = 1:count-1 %for every interval between 2 local minima
    i1 = local_min(point,2);%index number of local minima, with respect to the whole track map
    i2 = local_min(point+1,2);
    gap = i2-i1;
   
    for i = 0:local_min(point,2) %for every point in the interval
        target = local_min(point+1,2) - i; % ACTUAL index of iteration(start from the last point of interval)
        curv = abs(1/C2(target-1));
        
        % differentiate straight line and corner aero properties
        if curv >30
            CL = CLs; 
            CD = CDs;
        else
            CL = CLc;
            CD = CDc;
        end

        distance = dist(target)-dist(target-1);%distance between previous point and the local minimum

        v1 = final_lsp(target-1);
        v2 = final_lsp(target);        
        SA = 0;

        % Calculate velocity after braking with these conditions    
        braked_v = calculateBrakeV(v1,v2,distance,camber,SA,tyre_model,mass,air_density,frontel_area,CL,CD,tc_long,sen_long,phit,P,useMode);
                   
        final_lsp(target-1) = braked_v;                 
    end     
   
end
end



