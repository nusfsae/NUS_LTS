%this function determines max velocity the tire can give

function vel = v_calculater(tyre_model,mass,air_density,frontel_area,coef_lift,curv,camber,slip_ang,max_speed)

vel = max_speed;

for i = 0:0.1:max_speed

    Reaction_f = mass*9.81 + 0.5*air_density*frontel_area*coef_lift*(i^2); %reaction force of car with aero effect
    [Lat,~,~] = tyres(camber,slip_ang,Reaction_f,tyre_model); %max lateral force from tyre given normal load on tyre
    Lat = 0.67*Lat;

    corner_f = (mass*i^2)/curv; %required force for car to corner

    if corner_f > Lat %if required force to corner higher than max force tyre can provide
        vel = i-0.1; %
        
        break
    end
    
end

end