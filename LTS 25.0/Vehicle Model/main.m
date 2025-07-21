% Main Lap Time Simulation written in one single function

function [sim,lap_time_sim,lapsetime] = main( ...
    mass,C2,dist,pos,air_density,tyre_model,P,long_slip,phit,CLc,CLs,CDc,CDs,frontel_area,camber,del_max,max_rpm, ...
    max_torque,FDR,R_wheel,tc_lat,tc_long,sen_lat,sen_long,wheelbase,rollingstart,useMode,Ipeak,cg_h,Long_Accel,ab)

% fprintf("Initiating Simulation... ..."+"\n");

dist1 = dist;

if rollingstart == 1
    len=length(C2);
    C2 = horzcat(C2,C2,C2);
    pos = horzcat(pos,pos,pos);
    dist2 = dist1+2*dist(len)-dist(len-1);
    dist3 = dist2+2*dist(len)-dist(len-1);
    dist = horzcat(dist,dist2,dist3);
    Long_Accel = horzcat(Long_Accel,Long_Accel,Long_Accel);
end

fprintf("Generating Boundary Speed Profile... ..."+"\n");

% Initialize simulation results
num = length(C2);
sim = struct();
sim.latG = zeros(num,1);
sim.speed = zeros(num,1);
sim.torque = zeros(num,1);
sim.accel = zeros(num,1);
sim.brake = zeros(num,1);
sim.longG = zeros(num,1);
sim.throttle = zeros(num,1);



% Generate boundary speed profile

[sim] = cornerProfile(mass,C2,air_density,frontel_area,CLc,tyre_model,camber,max_rpm,FDR,R_wheel, ...
    tc_lat,sen_lat,phit,P,wheelbase,del_max,cg_h,Long_Accel,ab,sim);%output a nx2 array
fprintf("Almost there... ..."+"\n");
assignin('base','sim1',sim);

% Generate Acceleration Speed Profile
[sim] = accelProfile(dist,C2,sim,mass,air_density,frontel_area,CLs,CLc,CDs, ...
    CDc,camber,tyre_model,FDR,R_wheel,max_rpm,tc_long,sen_long,phit,P,Ipeak,Long_Accel,cg_h,wheelbase,ab);
assignin('base','sim2',sim);
% Generate Limit Speed Profile
sim = brakeProfile(C2,dist,camber,tyre_model,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,sim, ...
    tc_long,sen_long,phit,P,useMode,Long_Accel,cg_h,wheelbase,ab);
assignin('base','sim3',sim);
if rollingstart == 1
    len=length(C2);   
    sim.latG= sim.latG(len/3+1:len*2/3);
    sim.speed= sim.speed(len/3+1:len*2/3);
    sim.torque=sim.torque(len/3+1:len*2/3);
    sim.accel=sim.accel(len/3+1:len*2/3);
    sim.brake= sim.brake(len/3+1:len*2/3);
    sim.longG= sim.longG(len/3+1:len*2/3);
    sim.throttle= sim.throttle(len/3+1:len*2/3);
end


% Return lap time simulation, lapsetime: lap time at each data point
[lap_time_sim,lapsetime] = Lap_Time_Simulation(sim,dist1);

% Calculate telemetry data
sim = longGTele(sim,dist1); %return Long G diagram
sim = latGTele(sim,C2); %return Lat G diagram
sim = throttleTele(sim,dist1,max_torque,FDR,R_wheel,mass);
sim = brakeTele(sim,dist1,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,C2,tyre_model,P,tc_long);


fprintf("Simulation completed!" +"\n");

fprintf("Simulated Lap Time is "+lap_time_sim+" seconds"+".\n");

end
