% This function returns lap time simulation in seconds
function [lap_time_sim,lapsetime] = Lap_Time_Simulation(sim,dist)

lap_time_sim = 0;
lapsetime = zeros(length(sim.speed),1);

for point = 1:length(sim.speed)-1
    gap = dist(point+1) - dist(point); %distance between 2 points
    speed = (sim.speed(point+1,1) + sim.speed(point,1))/2; %average speed at every interval
    
    lap_time_sim = lap_time_sim + (gap/speed);
    lapsetime(point) = lap_time_sim;

end

lapsetime(length(sim.speed)) = lap_time_sim;

end