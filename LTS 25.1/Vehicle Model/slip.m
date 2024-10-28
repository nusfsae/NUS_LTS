function new_alpha = slip(mass,air_density,frontel_area,coef_lift,v,camber,alpha,tyre_model,C2)

    Reaction_f = (mass*9.81 + 0.5*air_density*frontel_area*coef_lift*(v^2))/4; %for one tyre
    [Lat,Long,~] = tyres(camber,alpha,Reaction_f,tyre_model);
    CS = 21000;

    new_alpha = (Lat/CS)*(60/(2*pi));

end