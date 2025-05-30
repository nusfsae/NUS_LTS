%this function determines max velocity the tire can give

function [vel,SA_f] = calculateCornerV(tyre_model,mass,air_density,frontel_area,CLc,curv,IA,slip_ang,max_rpm,FDR,R, ...
    tc_lat,sen_lat,phit,P,wheelbase,del_max,cg_h,longG,ab)

max_speed = (max_rpm/FDR)*pi*2*R/60;
vel = max_speed;

R = curv;
del_ack = rad2deg(wheelbase/R); % ackerman steering angle

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
    
    % define the smaller Fz
    Fz_s = Fz_r;
    Fz_b = Fz_f; 

    if Fz_f<Fz_r
        Fz_b = Fz_r;
        Fz_s = Fz_f;
    end
    
    % limit maximum slip angle with max steering angle
    if del_max-del_ack>0
        max_SA = del_max-del_ack;
    else
        max_SA = 0;
    end

    % find SA for lighter loaded tire
    for SA = 0:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_s,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_s,SR,SA+0.1,IA,P,V);

        SA_optimized_s = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_s = SA;            
            break
        end
    end  
    
    % find SA for heavier loaded tire
    for SA = SA_optimized_s:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_b,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_b,SR,SA+0.1,IA,P,V);
        
        SA_optimized_b = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_b = SA;      
            break
        end
    end      
    
    % assign slip angle values to front and rear tires
    if Fz_f<Fz_r
        SA_f = SA_optimized_s;
        SA_r = SA_optimized_b;
    else
        SA_f = SA_optimized_b;
        SA_r = SA_optimized_s;
    end
    
    % limit front slip angle with steering angle
    if SA_f > max_SA
        SA_f = max_SA;
    end

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

del = del_ack + SA_f;

end