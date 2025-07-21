%this function determines max cornereing velocity the tire can give

function [vel] = calculateCornerV(tyre_model,mass,air_density,frontel_area,CLc,curv,IA,max_rpm,FDR,R, ...
    tc_lat,sen_lat,phit,P,wheelbase,del_max,cg_h,longG,ab)

max_speed = (max_rpm/FDR)*pi*2*R/60;
vel = max_speed;

R = curv;
SR = 0;

for V = 0.1:0.1:40
    Weight = mass*9.81;
    Downforce = 0.5*air_density*frontel_area*CLc*V^2;
    Fz = Weight+Downforce;

    % amount of load transfer
    delta_W = Fz*longG*cg_h/wheelbase;

    % adjust normal load with load transfer
    Fz_f = (Weight + Downforce*ab)/4 - (delta_W)/2;
    Fz_r = (Weight + Downforce*ab)/4 + (delta_W)/2;         
          
    SA_f = 10;
    SA_r = 10;

    % obtain tire forces with front and rear loads
    [Fy_f,~] = tires(tyre_model,Fz_f,SR,SA_f,IA,P,V);
    [Fy_r,~] = tires(tyre_model,Fz_r,SR,SA_r,IA,P,V);

    Fy_f = Fy_f*2*tc_lat*sen_lat;
    Fy_r = Fy_r*2*tc_lat*sen_lat;
   
    Lat = Fy_f + Fy_r;

    % required force for car to corner
    corner_f = (mass*V^2)/R; 

    % if required force to corner higher than max force tyre can provide
    if corner_f > Lat 
        vel = V-0.1; 
        
        break
    end
end

end