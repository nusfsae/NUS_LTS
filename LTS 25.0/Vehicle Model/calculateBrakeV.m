%this function finds previous point max speed under braking scenario

%given velocity of current point, find max speed of the previous point
%v2 is speed of current point, v1 is upper limit of previous point speed
%(v1>v2)
function velocity = calculateBrakeV(v1,v2,dist,camber,slip_ang,tyre_model,mass,air_density,frontel_area,CL,CD,tc_long,sen_long,phit,P,useMode)
velocity = v1;
P = convpres(P, 'psi', 'Pa'); % convert psi to pa

for vel = v1:-0.1:v2
    
    Reaction_f = 0.25*(mass*9.81 + 0.5*air_density*frontel_area*CL*(vel^2));%normal load under this speed at one tire
    Drag = 0.5*air_density*frontel_area*CD*(vel^2);

    
    alpha = 0; % convert deg to rad
    long_slip = 0.1;
    
    V = 10;    

    inputsMF = [Reaction_f long_slip alpha camber phit V P];
    %disp(inputsMF)

    [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
    
    Long = 4*tc_long*sen_long*abs(outMF(1));%Tire correlation factor
    %disp(Long)
 
    deccel = (Long+Drag)/mass;%deceleration value        
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