%this function return velocity profile after considering car acceleration
%this function DONT require acceleration function
function accel_profile = accel_lsp_v2(dist,C2,BSP,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,tyre_model)
track_len = length(C2);
accel_profile = zeros(track_len,1);
slip_ang = slip_angle(C2);%row matrix slip_ang for this track
%zero speed at the start




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

    Reaction_f = 0.25*(mass*9.81+0.5*air_density*frontel_area*CL*vel^2);
    [~,Long,~] = tyres(camber,SA,Reaction_f,tyre_model);
    Long = 4*0.67*Long;

    Drag = 0.5*air_density*(vel^2)*CD*frontel_area;%drag force at this speed
    Tractive = Long-Drag;

    avail_accel = Tractive/mass; %calculate available acceleration remained
    
    if avail_accel > 8 %if tyre accel larger than max accel by powertrain
        avail_accel = 8; %max accel limited by powertrain
    end

    v_accel = sqrt((accel_profile(point))^2 + 2*avail_accel*(dist(point+1)-dist(point))); %calculate max speed after accel
            
    if v_accel > BSP(point+1) %ensure the car wont exceed max corner speed
        accel_profile(point+1) = BSP(point+1);
                            
    else                
       accel_profile(point+1) = v_accel; %full acceleration
                           
    end
       
end
end




