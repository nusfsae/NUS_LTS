
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

%% run performance envelope
GGV;

%% dynamic simulation
dynamics;

%% plotting

figure

tiledlayout(3,1);
nexttile;
% plot speed profile
plot(dist,sim.speed*3.6);ylabel('speed (km/h)');
% plot ax/ay
nexttile
plot(dist,sim.ay/9.81);ylabel('ay (G)');
nexttile
plot(dist,sim.ax/9.81);ylabel('ax (G)');
xlabel('distance(m)');

