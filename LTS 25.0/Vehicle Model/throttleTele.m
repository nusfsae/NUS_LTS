function sim = throttleTele(sim,dist,max_torque,FDR,R,mass)
simLong = longGTele(sim,dist);
Long_Accel = simLong.longG;
len = length(sim.speed);

for i = 1:len
    actual = 0;

    if Long_Accel(i)>0
        actual = Long_Accel(i);    
    end
    % Powertrain maximum tractive force
    if sim.speed(i)<23.6298
        torque = max_torque;
    else
        torque = -5.4*sim.speed(i) + 297.181;
    end
    F_powertrain = torque*FDR/R;
    
    full = F_powertrain/mass/9.81;    
    tt= (actual/full)*100;
    if tt>100
        tt =100;
    end
    sim.throttle(i) = tt; 
end
end