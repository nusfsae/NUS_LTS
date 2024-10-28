% Use this function to compare simulation with MoTec Data

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\LTS25\Vehicle Model')

run control_panel;
close all


cd('C:\Users\PC5\Documents\Patrick\EG3301R')
motec = load('241013 JTC Lap 2 Motec');
speed = motec.Corr_Speed.Value;
time = motec.Corr_Speed.Time;
distance = motec.Corr_Dist.Value;
distance = distance - distance(1);
time = time - time(1);


% Compare speed
plot(distance,speed,'DisplayName', 'MoTec Data');
xlabel("Distance (m)")
ylabel("Speed (km/h)")
sgtitle("Speed Profile Comparison")
hold on
plot(dist1,final_lsp*3.6,'DisplayName', 'Simulation Data')
legend


% Compare Lap Time
figure
plot(distance,time,'DisplayName', 'Test Run Data')
xlabel("Distance (m)")
ylabel("Time (s)")
sgtitle("Lap Time Comparison")
hold on
plot(dist1,lapsetime,'DisplayName', 'Simulation Data')
legend
