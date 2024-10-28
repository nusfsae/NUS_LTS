%this function determines max velocity the tire can give

function vel = v_calculater(tyre_model,mass,air_density,frontel_area,CLc,curv,camber,max_speed,steer_max)

vel = max_speed;
finishnow = false;

for v = max_speed:-0.1:0
    for steer = 0:0.5:steer_max
        slipang = 0.5*steer;
        if slipang>10
            slipang = 10;
        end
        Reaction_f = 1/4*(mass*9.81 + 0.5*air_density*frontel_area*CLc*(v^2));
        [Lat,Long,~] = tyres(camber,slipang,Reaction_f,tyre_model);
        Fy = 4*(Long*sin(deg2rad(slipang))+Lat*cos(deg2rad(slipang)));

        corner_f = (mass*v^2)/curv; %required force for car to corner

        if Fy>corner_f %if required force to corner higher than max force tyre can provide
            vel = v; %
            finishnow = true;
        
            break
        end

        
    end
    if finishnow
        break
    end
    
end



end