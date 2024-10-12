%this function return velocity profile after considering car acceleration
%this function DONT require acceleration function
function accel_profile = accel_lsp_v2(dist,C2,pos,BSP,mass,air_density,frontel_area,coef_lift,coef_drag,camber,tyre_model)
track_len = length(C2);
accel_profile = zeros(track_len,1);
slip_ang = slip_angle(C2);%row matrix slip_ang for this track
%zero speed at the start




for point = 1:(track_len-1)
    SA = slip_ang(point);%slip angle at this point
    vel = accel_profile(point); 

    Reaction_f = mass*9.81+0.5*air_density*frontel_area*coef_lift*vel^2;
    [~,Long,~] = tyres(camber,SA,Reaction_f,tyre_model);

    Drag = 0.5*air_density*(vel^2)*coef_drag*frontel_area;%drag force at this speed
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




