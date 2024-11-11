%this function determines max velocity the tire can give

function vel = v_calculater(tyre_model,mass,air_density,frontel_area,CLc,curv,camber,slip_ang,max_rpm,FDR,R,tc_lat,sen_lat)

max_speed = (max_rpm/FDR)*pi*2*R/60;
vel = max_speed;

for i = 0:0.1:max_speed
    %reaction force of car with aero effect
    Reaction_f = 0.25*(mass*9.81 + 0.5*air_density*frontel_area*CLc*(i^2)); 
    %max lateral force from tyre given normal load on one tyre
    [Lat,~,~] = tyres(camber,slip_ang,Reaction_f,tyre_model); 
    Lat = 4*tc_lat*sen_lat*Lat; 
    %required force for car to corner
    corner_f = (mass*i^2)/curv; 
    %if required force to corner higher than max force tyre can provide
    if corner_f > Lat 
        vel = i-0.1; 
        
        break
    end
    
end

end