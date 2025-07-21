% Use this function to compare simulation with MoTec Data

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')

run control_panel;


% Load data from MoTec
%cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Motec Data')
%motec2 = load('241013 JTC Lap 2 Motec');
%speed2 = motec2.Corr_Speed.Value;
%time2 = motec2.Corr_Speed.Time;
%distance2 = motec2.Corr_Dist.Value;
%distance2 = distance2 - distance2(1);
%time2 = time2 - time2(1);

% Load data from MoTec
%cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Motec Data')
%motec4 = load('241013 JTC Lap 4 Motec');
%speed4 = motec4.Corr_Speed.Value;
%time4 = motec4.Corr_Speed.Time;
%distance4 = motec4.Corr_Dist.Value;
%distance4 = distance4 - distance4(1);
%time4 = time4 - time4(1);

% Load data from MoTec
%cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Motec Data')
%motec6 = load('241013 JTC Lap 6 Motec');
%speed6 = motec6.Corr_Speed.Value;
%time6 = motec6.Corr_Speed.Time;
%distance6 = motec6.Corr_Dist.Value;
%distance6 = distance6 - distance6(1);
%time6 = time6 - time6(1);

% Load data from MoTec
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Motec Data')
% motec8 = load('Accel 25 49134.mat');
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

% compare longG
% figure
% plot(dist,accel8);
% hold on
% plot(dist,sim.longG,'DisplayName', 'Sim');

% Compare speed
figure
plot(distance8,speed8,'DisplayName', 'Khai');
xlabel("Distance (m)")
ylabel("Speed (km/h)")
ylim([0 130])
sgtitle("Speed Profile Validation")


%hold on
%plot(distance4,speed4,'DisplayName', 'MoTec Data 2');

%hold on
%plot(distance8,speed6,'DisplayName', 'MoTec Data 3');
 
hold on
plot(dist,sim.speed*3.6,'DisplayName', 'Sim');

legend

% Compare Lap Time
%figure
%plot(distance2,time2,'DisplayName', 'Test Run Data 1')
%hold on
%plot(distance4,time4,'DisplayName', 'Test Run Data 2')
%hold on
%plot(distance8,time8,'DisplayName', 'Test Run Data 3')

%xlabel("Distance (m)")
%ylabel("Time (s)")
%sgtitle("Lap Time Validation")
%hold on
%plot(dist1,lapsetime,'DisplayName', 'Simulation Data')
%legend
