%this function return velocity profile after considering car acceleration
function accel_profile = accelProfile(dist,C2,BSP,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,SAprofile,camber, ...
    tyre_model,FDR,R,max_torque,tc_long,sen_long,phit,P,Ipeak,Long_Accel,cg_h,wheelbase,ab)

track_len = length(C2);
accel_profile = zeros(track_len,1);
%slip_ang = slip_angle(C2); % estimate slip angle for this track

for point = 1:(track_len-1)

    SA = SAprofile(point);    
    longG = Long_Accel(point);
    vel = accel_profile(point); 
    curv = abs(1/C2(point));
    V = vel;

    if curv > 30
        CL = CLs;
        CD = CDs;
    else
        CL = CLc;
        CD = CDc;
    end

    Weight = mass*9.81;
    Downforce = 0.5*air_density*frontel_area*CL*V^2;
    Fz = Weight+Downforce;

    % amount of load transfer
    delta_W = Fz*longG*cg_h/wheelbase;

    % adjust normal load with load transfer
    Fz_f = (Weight + Downforce*ab)/4 - (delta_W)/2;
    Fz_r = (Weight + Downforce*ab)/4 + (delta_W)/2;


    % Tyre maximum tractive force

    alpha = SA;
    long_slip = 0.1;
    V = 10;   

    [~,Fx_f] = tires(tyre_model,Fz_f,alpha,long_slip,camber,P,V); 
    [~,Fx_r] = tires(tyre_model,Fz_r,alpha,long_slip,camber,P,V);     
    
    Long = sen_long*Fx_r*2;
    
    Drag = 0.5*air_density*(vel^2)*CD*frontel_area;%drag force at this speed
    F_t= Long-Drag;
    
    % Powertrain maximum tractive force
    F_powertrain = 0.9*Ipeak*220*FDR/R;

    %calculate available acceleration remained
    F = min(F_t,F_powertrain);
    avail_accel = F/mass; 
    
    %calculate max speed after accel
    v_accel = sqrt((accel_profile(point))^2 + 2*avail_accel*(dist(point+1)-dist(point))); 
 
    %ensure the car wont exceed max corner speed
    if v_accel > BSP(point+1) 
        accel_profile(point+1) = BSP(point+1);                            
    else                
       accel_profile(point+1) = v_accel; %full acceleration                           
    end       
end
end




