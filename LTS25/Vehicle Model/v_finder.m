%this function finds previous point max speed under braking scenario

%given velocity of current point, find max speed of the previous point
%v2 is speed of current point, v1 is upper limit of previous point speed
%(v1>v2)
function velocity = v_finder(v1,v2,dist,camber,slip_ang,tyre_model,mass,air_density,frontel_area,coef_drag)
velocity = v1;

for vel = v1:-0.1:v2
    Reaction_f = mass*9.81 + 0.5*air_density*frontel_area*coef_drag*(vel^2);%normal load under this speed
    [~,Long,~] = tyres(camber,slip_ang,Reaction_f,tyre_model);%longitudinal force by tyres under this conditions
    Long = 0.67*Long; %Tire correlation factor
    deccel = Long/mass;%deceleration value
        
    new_v2 = sqrt((vel^2) - 2*deccel*dist);%

    

    if new_v2 < v2
        velocity = vel;
        %disp(deccel/9.81);
        break
    else
        continue
     
    end
end

if velocity>v1
    velocity = v1;
end

end