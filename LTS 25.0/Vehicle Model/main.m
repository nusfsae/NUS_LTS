% Main Lap Time Simulation written in one single function

function [BSP,Accel_LSP,Final_LSP,lap_time_sim,lapsetime,Long_Accel,Lat_Accel,throttle_graph,brake_graph] = main( ...
    mass,C2,dist,pos,air_density,tyre_model,P,long_slip,phit,CLc,CLs,CDc,CDs,frontel_area,camber,maxsteer,max_rpm, ...
    max_torque,FDR,R_wheel,tc_lat,tc_long,sen_lat,sen_long,wheelbase,rollingstart,useMode)

fprintf("Initiating Simulation... ..."+"\n");

dist1 = dist;

if rollingstart == 1
    len=length(C2);
    C2 = horzcat(C2,C2,C2);
    pos = horzcat(pos,pos,pos);
    dist2 = dist1+2*dist(len)-dist(len-1);
    dist3 = dist2+2*dist(len)-dist(len-1);
    dist = horzcat(dist,dist2,dist3);    
end

% Generate boundary speed profile
fprintf("Generating Boundary Speed Profile... ..."+"\n");
BSP = cornerProfile(mass,C2,air_density,frontel_area,CLc,tyre_model,camber,max_rpm,FDR,R_wheel,tc_lat,sen_lat,phit,P,useMode);%output a nx2 array
fprintf("Almost there... ..."+"\n");

% Generate Acceleration Speed Profile
Accel_LSP = accelProfile(dist,C2,BSP,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,tyre_model,FDR,R_wheel,max_torque,tc_long,sen_long,phit,P,useMode);

% Generate Limit Speed Profile
Final_LSP = brakeProfile(C2,dist,camber,tyre_model,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,Accel_LSP,tc_long,sen_long,phit,P,useMode);


if rollingstart == 1
    len=length(C2);
    BSP = BSP(len/3+1:len*2/3);
    Accel_LSP = Accel_LSP(len/3+1:len*2/3);
    Final_LSP = Final_LSP(len/3+1:len*2/3);
end

% Return lap time simulation, lapsetime: lap time at each data point
[lap_time_sim,lapsetime] = Lap_Time_Simulation(Final_LSP,dist1);

% Calculate telemetry data
Long_Accel = longGTele(Final_LSP,dist1); %return Long G diagram
Lat_Accel = latGTele(Final_LSP,C2); %return Lat G diagram
throttle_graph = throttleTele(Final_LSP,dist1,max_torque,FDR,R_wheel,mass);
brake_graph = brakeTele(Final_LSP,dist1,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,C2,tyre_model,P,tc_long);

clc
fprintf("Simulation completed!" +"\n");

fprintf("Simulated Lap Time is "+lap_time_sim+" seconds"+".\n");

end