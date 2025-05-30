%this function determines max velocity the tire can give

function vel = new_calculateCornerV(tyre_model,mass,air_density,frontel_area,CLc,curv,camber,slip_ang,max_rpm,FDR,R,tc_lat,sen_lat,phit,P,useMode)

max_speed = (max_rpm/FDR)*pi*2*R/60;
vel = max_speed;
P = convpres(P, 'psi', 'Pa'); % convert psi to pa

for i = max_speed:-0.1:0.1
    %reaction force of car with aero effect
    Reaction_f = 0.25*(mass*9.81 + 0.5*air_density*frontel_area*CLc*(i^2)); %per tire
    slip_ang = 10;
    long_slip = 0;
    alpha = deg2rad(slip_ang); % convert deg to rad
    
    V = i;
    

    inputsMF = [Reaction_f long_slip alpha camber phit V P];
    
    % find max lateral force from two wheel combo

    [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
    Lat = abs(outMF(2));
    Lat = 4*tc_lat*sen_lat*Lat; 

    %required force for car to corner
    corner_f = (mass*i^2)/curv; 
    %if required force to corner less than max force tyre can provide
    if corner_f < Lat 
        vel = i;         
        break
    end
    
end

end

