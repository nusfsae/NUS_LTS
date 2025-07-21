%this function finds previous point max speed under braking scenario

%given velocity of current point, find max speed of the previous point
%v2 is speed of current point, v1 is upper limit of previous point speed
%(v1>v2)
function velocity = calculateBrakeV(v1,v2,dist,camber,slip_ang,tyre_model,mass,air_density, ...
    frontel_area,CL,CD,tc_long,sen_long,phit,P,useMode,longG,cg_h,wheelbase,ab,SA)

velocity = v1;

for vel = v1:-0.1:v2
    Weight = mass*9.81;
    Downforce = 0.5*air_density*frontel_area*CL*vel^2;
    Fz = Weight+Downforce;

    % amount of load transfer
    delta_W = Fz*longG*cg_h/wheelbase;

    % adjust normal load with load transfer and aero balance
    Fz_f = (Weight + Downforce*ab)/4 - (delta_W)/2;
    Fz_r = (Weight + Downforce*ab)/4 + (delta_W)/2;

    % Tyre maximum tractive force
    alpha = 0;
    long_slip = 0.1;

    [~,Fx_f] = tires(tyre_model,Fz_f,long_slip,alpha,camber,P,10);
    [~,Fx_r] = tires(tyre_model,Fz_r,long_slip,alpha,camber,P,10);   

   
    Drag = 0.5*air_density*frontel_area*CD*(vel^2);
    
    Long = (Fx_f + Fx_r) * tc_long*sen_long;  % does not make sense cuz u should have 4 wheel brake but i divide by 2 to make number realistic
        
    deccel = (Long+Drag)/mass; % deceleration value        
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