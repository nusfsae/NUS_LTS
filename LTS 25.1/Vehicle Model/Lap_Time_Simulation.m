%this function returns lap time simulation in seconds
function [lap_time_sim,lapsetime] = Lap_Time_Simulation(final_lsp,dist)

lap_time_sim = 0;
lapsetime = zeros(length(final_lsp),1);

for point = 1:length(final_lsp)-1
    gap = dist(point+1) - dist(point); %distance between 2 points
    speed = (final_lsp(point+1) + final_lsp(point))/2; %average speed at every interval
    
    lap_time_sim = lap_time_sim + (gap/speed);
    lapsetime(point) = lap_time_sim;

end

lapsetime(length(final_lsp)) = lap_time_sim;

end