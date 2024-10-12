%this function returns lap time simulation in seconds
function lap_time_sim = Lap_Time_Simulation(final_lsp,dist)

lap_time_sim = 0;
track_len = length(dist);


for point = 1:track_len-1
    gap = dist(point+1) - dist(point);%distance between 2 points
    speed = (final_lsp(point+1) + final_lsp(point))/2;%average speed at every gap
   
    lap_time_sim = lap_time_sim + (gap/speed);

end

