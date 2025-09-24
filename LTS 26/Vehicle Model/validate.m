% validation

% Load data from MoTec
motec8 = load('Endurance 25 fastest.mat');
speed8 = motec8.Corr_Speed.Value;
time8 = motec8.Corr_Speed.Time;
distance8 = motec8.Corr_Dist.Value;
distance8 = distance8 - distance8(1);
time8 = time8 - time8(1);
accel8 = zeros(length(speed8),1);
for i = 1:length(speed8)-1
    accel8(i) = (((speed8(i+1)/3.6)^2 - (speed8(i)/3.6)^2)/(2*(distance8(i+1)-distance8(i))))/9.81;    
end
accel8(length(speed8)) = accel8(length(speed8)-1);

% Compare speed
figure
plot(distance8,speed8,'DisplayName', 'Khai');
xlabel("Distance (m)")
ylabel("Speed (km/h)")
ylim([0 130])
sgtitle("Speed Profile Validation")
hold on
plot(dist,sim.speed*3.6,'DisplayName', 'Sim');