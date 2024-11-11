% Main Lap Time Simulation written in one single function

function [lap_time_sim,final_lsp] = main(mass,C2,dist,pos,air_density,frontel_area,CLc,CLs,CDc,CDs,tyre_model,camber,max_rpm,max_torque,FDR,R_wheel,tc_lat,tc_long,sen_lat,sen_long,wheelbase,maxsteer,rollingstart)

dist1 = dist;
C21 = C2;
pos1 = pos;

if rollingstart == 1
    len=length(C2);
    C2 = horzcat(C2,C2,C2);
    pos = horzcat(pos,pos,pos);
    dist2 = dist1+2*dist(len)-dist(len-1);
    dist3 = dist2+2*dist(len)-dist(len-1);
    dist = horzcat(dist,dist2,dist3);    
end

% Generate boundary speed profile
BSP = bsp(mass,C2,air_density,frontel_area,CLc,tyre_model,camber,max_rpm,FDR,R_wheel,tc_lat,sen_lat);%output a nx2 array

% Estimate slip angle base on track 
slip_ang = slip_angle(C2);

% Generate Acceleration Speed Profile
Accel_LSP = accel_lsp_v2(dist,C2,BSP,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,tyre_model,FDR,R_wheel,max_torque,tc_long,sen_long);

% Generate Limit Speed Profile
temp_lsp = lsp(C2,dist,camber,tyre_model,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,Accel_LSP,tc_long,sen_long);

% Limit by maximum yaw rate
[final_lsp,yaw_diagram] = yawcal(temp_lsp,C2,maxsteer,wheelbase);

if rollingstart == 1
    len=length(C2);
    BSP = BSP(len/3:len*2/3-1);
    Accel_LSP = Accel_LSP(len/3:len*2/3-1);
    final_lsp = final_lsp(len/3:len*2/3-1);
end

% Return lap time simulation, lapsetime: lap time at each data point
[lap_time_sim,lapsetime] = Lap_Time_Simulation(final_lsp,dist1);
end