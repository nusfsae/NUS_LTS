%this function finds previous point max speed under braking scenario

%given velocity of current point, find max speed of the previous point
%v2 is speed of current point, v1 is upper limit of previous point speed
%(v1>v2)
function velocity = calculateBrakeV1w(v1,v2,dist,camber,slip_ang,tyre_model,mass,air_density,frontel_area,CL,CD,tc_long,sen_long,phit,P,useMode)
velocity = v1;

for vel = v1:-0.1:v2
    
    Reaction_f = 0.25*(mass*9.81 + 0.5*air_density*frontel_area*CL*(vel^2));%normal load under this speed at one tire
    Drag = 0.5*air_density*frontel_area*CD*(vel^2);
    
    alpha = 0; % assume no sideslip when braking
    long_slip = 0.1; % assume braking at optimized SR    
    V = 10; % anyhow give a number to tire model    
    
    [Lat,Long] = tires(tyre_model,Reaction_f,long_slip,alpha,camber,P,10);
    
    Long = 4*tc_long*sen_long*Long;% Tire correlation factor
     
    deccel = (Long+Drag)/mass; % deceleration value        
    new_v2 = sqrt((vel^2) - 2*deccel*dist);   
    
    if new_v2 < v2
        velocity = vel;
        
        break
    end
end

if velocity>v1
    velocity = v1;
end

end