% Path Reconstruction 2.0
% Theory : Kinematic point mass model
% Raw data: Vehicle Speed, Vehicle Rotation Mid yaw Rate
% Note : 
% 1) High sampling rate required for assumption of contant speed over a small
% distance, 100HZ (native sampling rate of MOTEC) is recommended
% 2) Positive yaw in MOTEC is in clockwise direction

% Load data, convert to SI unit, and calculate time step
cd('D:\Patrick\VD SIM\LTS25\Track Model')
load ('24 Endurance Fastest Motec.mat');

Corr_Speed.Value = Corr_Speed.Value * 1000 / 3600;
G_Sensor_Front_Yaw_Rate.Value = G_Sensor_Front_Yaw_Rate.Value * pi / 180;
t = Corr_Speed.Time(2) - Corr_Speed.Time(1);

% Create Structure for position,velocity and theta
% Set initial condition (by default, all 0)
size = length(Corr_Speed.Time);
pos = struct('x', zeros(size,1), 'y', zeros(size,1));
vel = struct('x',zeros(size,1),'y',zeros(size,1));
theta = zeros(size,1);
dist = zeros(size,1);
C1 = zeros(size,1);
C2 = zeros(size,1);
C3 = zeros(size,1);

% Rotate the track
theta(1) = -100/180*pi;

% Car Position Tracing (main loop)
[vel.x(1), vel.y(1), theta(1)] = next_orientation (Corr_Speed.Value(1), G_Sensor_Front_Yaw_Rate.Value(1), theta(1), t);
[pos.x(2), pos.y(2)] = next_coordinate (vel.x(1), vel.y(1), pos.x(1), pos.y(1), t);
for i = 3:size
    [vel.x(i-1), vel.y(i-1), theta(i-1)] = next_orientation (Corr_Speed.Value(i-1), G_Sensor_Front_Yaw_Rate.Value(i-2),theta(i-2), t);
    [pos.x(i), pos.y(i)] = next_coordinate (vel.x(i-1), vel.y(i-1), pos.x(i-1), pos.y(i-1), t);
end

% Curvature vs distance
for i = 1:size
    C1(i) = 1/(Corr_Speed.Value(i)/G_Sensor_Front_Yaw_Rate.Value(i));
    C3(i) = G_Sensor_Front_Yaw_Rate.Value(i)*9.81/Corr_Speed.Value(i)^2;
    if i>1
    dist(i) = ((Corr_Speed.Value(i)+Corr_Speed.Value(i-1))/2) * t + dist(i-1);
    end
end
C2 = movmean(C1,20);

% Plotting the map
%scatter (pos.x, pos.y)
figure
hold on
plot (dist,C1)
plot (dist,C2)

% creating distance based mesh
meshsize = 1; % metres
s = floor(dist(end)/meshsize);
Dist = 0 : meshsize : s*meshsize;
c2 = interp1(dist,C2,Dist);
%figure
%plot(Dist,c2)

% creating distance based map
x = interp1(dist,pos.x,Dist);
y = interp1(dist,pos.y,Dist);
figure
scatter(x,y)
axis equal

% Transfering distance based output
dist = Dist;
C2 = c2;
pos.x = x;
pos.y = y;

save("24 End Fs.mat",'pos','C2','dist')

clear x y C1 C3 c2 Dist i meshsize s size t vel theta

% Rotate the car speed into global x-y frame
function [Vx, Vy, new_theta] = next_orientation (v, yawRate, theta, t)
new_theta = theta - yawRate*t;
Vy = v * (sin(0.5*pi+new_theta));
Vx = v * (cos(0.5*pi+new_theta));
end

% Calculate the next position of car respect to global x-y frame
function [Dx,Dy] = next_coordinate (v_x, v_y, d_x, d_y, t)
Dx = d_x + v_x*t;
Dy = d_y + v_y*t;
end

