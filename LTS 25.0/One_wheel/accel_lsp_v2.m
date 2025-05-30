%this function return velocity profile after considering car acceleration
function accel_profile = accel_lsp_v2(dist,C2,BSP,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,tyre_model,FDR,R,max_torque,tc_long,sen_long)
track_len = length(C2);
accel_profile = zeros(track_len,1);
slip_ang = slip_angle(C2); % estimate slip angle for this track

for point = 1:(track_len-1)
    SA = slip_ang(point);%slip angle at this point
    vel = accel_profile(point); 
    curv = abs(1/C2(point));
    if curv > 30
        CL = CLs;
        CD = CDs;
    else
        CL = CLc;
        CD = CDc;
    end

    % Tyre maximum tractive force
    Reaction_f = 0.25*(mass*9.81+0.5*air_density*frontel_area*CL*vel^2);
    [~,Long,~] = tyres(camber,SA,Reaction_f,tyre_model);
    Long = 4*tc_long*sen_long*Long;
    Drag = 0.5*air_density*(vel^2)*CD*frontel_area;%drag force at this speed
    F_t= Long-Drag;
    
    % Powertrain maximum tractive force
    if vel<23.6298
        torque = max_torque;
    else
        torque = -5.4*vel + 297.181;
    end
    F_powertrain = torque*FDR/R;

    %calculate available acceleration remained
    F = min(F_t,F_powertrain);
    avail_accel = F/mass; 
    
    %calculate max speed after accel
    v_accel = sqrt((accel_profile(point))^2 + 2*avail_accel*(dist(point+1)-dist(point))); 
 
    %ensure the car wont exceed max corner speed
    if v_accel > BSP(point+1) 
        accel_profile(point+1) = BSP(point+1);                            
    else                
       accel_profile(point+1) = v_accel; %full acceleration                           
    end       
end
end




