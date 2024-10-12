% Path Reconstruction 1.0
% by using Inertial Measurement Method
% for educational purpose
% not for actual use due to stack-up error of inertial calculation

% Load data, convert to SI unit, and calculate time step
load ('skidpad_1.0.mat');
Lat_G.Value = Lat_G.Value * 9.81;
Long_G.Value = Long_G.Value * 9.81;
yaw_rate.Value = -yaw_rate.Value * pi / 180;
t = Lat_G.Time(2) - Lat_G.Time(1);

% Create Structure for position,velocity and acceleration
% Set initial condition (by default, all 0)
size = length(Lat_G.Time);
pos = struct('x', zeros(size,1), 'y', zeros(size,1),'time',{transpose(0:t:(size-1)*t)});
vel = struct('x',zeros(size,1),'y',zeros(size,1));
accel = struct('x',zeros(size,1),'y',zeros(size,1));
theta = zeros(size,1);

% Car Position Tracing (main loop)
[pos.x(2), pos.y(2), vel.x(2), vel.y(2)] = next_coordinate (Lat_G.Value(1), Long_G.Value(1), vel.x(1) ,vel.y(1) ,pos.x(1), pos.y(1), t,yaw_rate.Value(1));
for i = 3:size
    [accel.x(i-1), accel.y(i-1), theta(i-1)] = next_orientation (Long_G.Value(i-1), Lat_G.Value(i-1), yaw_rate.Value(i-2),theta(i-2), t);
    [pos.x(i), pos.y(i), vel.x(i), vel.y(i)] = next_coordinate (accel.x(i-1), accel.y(i-1), vel.x(i-1), vel.y(i-1), pos.x(i-1), pos.y(i-1), t,yaw_rate.Value(i-1));
end
% Map plotting
scatter (pos.x,pos.y);

% Rotate the Lat_G and Long_G into global x-y frame
function [Ax, Ay, new_theta] = next_orientation (long_G, lat_G, yawRate, theta, t)
new_theta = theta - yawRate*t;
Ay = long_G * cos(new_theta) - lat_G * (sin(new_theta));
Ax = long_G * sin(new_theta) + lat_G * (cos(new_theta));
end

% Caculate the next position of car respect to global x-y frame
function [Dx,Dy,Vx,Vy] = next_coordinate (Ax, Ay, v_x, v_y, d_x, d_y,t,yawrate)
Vx = v_x + Ax*t;
Vy = v_y + Ay*t;
Dx = d_x + v_x*t + 0.5*Ax*t^2;
Dy = d_y + v_y*t + 0.5*Ay*t^2;
end