% Driving patterns comparison

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')
%run control_panel

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0')
close all



% Load Lap 1
motec1 = load('Patrick Lap2'); 
speed1 = motec1.Corr_Speed.Value;
time1 = motec1.Corr_Speed.Time;
distance1 = motec1.Corr_Dist.Value;
throttle1 = motec1.Throttle_Pedal.Value;
brake1 = motec1.Brake_Pressure_Front.Value;


% Load Lap 2
motec2 = load('Ian Lap3'); 
speed2 = motec2.Corr_Speed.Value;
time2 = motec2.Corr_Speed.Time;
distance2 = motec2.Corr_Dist.Value;
throttle2 = motec2.Throttle_Pedal.Value;
brake2 = motec2.Brake_Pressure_Front.Value;


% scale back distance and time axis
dist1 = distance1 - distance1(1);
dist2 = distance2 - distance2(1);
t1 = time1 - time1(1);
t2 = time2 - time2(1);


% Compare speed
figure
plot(dist1,speed1,'DisplayName', 'Patrick');
xlabel("Distance (m)")
ylabel("Speed (km/h)")
sgtitle("Speed Profile Comparison")
hold on
plot(dist2,speed2,'DisplayName', 'Ian');
hold on 
%plot(dist,final_lsp*3.6,'DisplayName', 'LTS')
legend


% Compare Lap Time
figure
plot(dist1,t1,'DisplayName', 'Test Run Data')
xlabel("Distance (m)")
ylabel("Time (s)")
sgtitle("Lap Time Comparison")
hold on
plot(dist2,t2,'DisplayName', 'Test Run Data')
hold on
%plot(dist,lapsetime,'DisplayName', 'LTS')
legend


% Compare throttle inputs
figure
plot(dist1,throttle1,'DisplayName', 'Driver 1');
xlabel("Distance (m)")
ylabel("Throttle Input (%)")
sgtitle("Throttle Input Comparison")
hold on
plot(dist2, throttle2,'DisplayName', 'Driver 2');
hold on 
%plot(dist,throttle_graph,'DisplayName', 'LTS')
legend


% Compare brake inputs (No LTS for brake inputs)
figure
plot(dist1,brake1,'DisplayName', 'Driver 1');
xlabel("Distance (m)")
ylabel("Brake Input (psi)")
sgtitle("Brake Input Comparison")
hold on
plot(dist2, brake2,'DisplayName', 'Driver 2');
legend