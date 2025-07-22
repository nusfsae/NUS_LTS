%this function return velocity profile after considering car acceleration
function [sim] = accelProfile(dist,C2,sim,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber, ...
    tyre_model,FDR,R,max_rpm,tc_long,sen_long,phit,P,Ipeak,Long_Accel,cg_h,wheelbase,ab)

track_len = length(C2);
sim.accel(1,1) = 0;
sim.speed(1,1) = 0;

for point = 1:(track_len-1)
    
    longG = Long_Accel(point);
    
    vel = sim.accel(point,1); 
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
    alpha = 0;
    long_slip = 0.1;
      

    [~,Fx_r] = tires(tyre_model,Fz_r,long_slip,alpha,camber,P,10);     
    
    Long = sen_long*tc_long*Fx_r*2;
    
    Drag = 0.5*air_density*(vel^2)*CD*frontel_area;%drag force at this speed
    F_t= Long-Drag;
    
    % Powertrain maximum tractive force
    if vel<24.03  %24.03 % 360V setting
        F_powertrain = 0.6*0.8*Ipeak*220*FDR/R-Drag;
    else        
        v_max = (max_rpm/FDR)*pi*2*R/60; 
        % v_weak = 80/3.6; % field weakening start when speed = 80kmh for 333.75V setting
        v_weak = 86.5/3.6; % 360V setting
        Iweak = ((220-0)/(v_weak-v_max))*vel+220-((220-0)/(v_weak-v_max))*v_weak; 
        F_powertrain = 0.6*0.8*Ipeak*Iweak*FDR/R;
    end

    %calculate available acceleration remained
    F = min(F_t,F_powertrain);
    avail_accel = F/mass; 
    
    %calculate max speed after accel
    v_accel = sqrt((sim.accel(point,1))^2 + 2*avail_accel*(dist(point+1)-dist(point)));     
 
    %ensure the car wont exceed max corner speed
    if v_accel < sim.speed(point+1,1)              
       sim.speed(point+1,1) = v_accel;
       sim.accel(point+1,1) = v_accel;
    else
        sim.accel(point+1,1) = sim.speed(point+1,1);
    end  
    
    if avail_accel>0
        sim.torque(point,1) = avail_accel*mass*R;
    end
end
end




