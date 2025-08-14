
% open track model file
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\Github\NUS_LTS\LTS 26\Track Model')
track = '25 Endurance.mat'; % 24 Autocross % 25 Endurance % 241013 JTC PM % Skidpad_10m %JTC 2025 v1.mat %JTC v2.mat % JTC0504 % 75m Accel
load(track)

% open vehicle model directory
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\Github\NUS_LTS\LTS 26\Vehicle Model')

% initialize simulation results
num = length(C2);
sim = struct();
sim.ay = zeros(num,1);
sim.ax = zeros(num,1);
sim.speed = zeros(num,1);
sim.torque = zeros(num,1);
sim.accel = zeros(num,1);
sim.brake = zeros(num,1);
sim.throttle = zeros(num,1);

% run simulation
GGD;

%%

figure
corner;

% plot BSP
plot(dist,sim.speed);

drivebrake;

% plot speed profile
hold on;plot(dist,sim.speed);
% plot ax/ay
figure
plot(dist,sim.ay/9.81);yyaxis left;ylabel('ay');
hold on; plot(dist,sim.ax/9.81);yyaxis right;ylabel('ax');
