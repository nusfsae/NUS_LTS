%calculate max acceleration of vehicle
%both corner and straight considered


function avail_accel = acceleration(vel,mass,air_density,frontel_area,coef_lift,coef_drag,coef_friction,C2)

curv = 1/C2;
Reaction_f = mass*9.81+0.5*air_density*frontel_area*coef_lift*vel^2; %reaction force with aero downforce

tractive = coef_friction*Reaction_f;
corner_f = mass*(vel^2)/curv ;
drag = 0.5*air_density*frontel_area*coef_drag*(vel^2);

if curv > 200 %radius too large, track is straight
    corner_f = 0;
end

avail_accel = (sqrt(tractive^2-corner_f^2)-drag)/mass;

end

