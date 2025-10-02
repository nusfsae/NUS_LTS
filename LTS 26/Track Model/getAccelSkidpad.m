% generate accel/skidpad

%% skidpad

R = 9.125;  % Radius in meters (15.25m diameter / 2 + 0.75m offset)
size = 400;  % Points around the circle

% Initialize arrays
Pos = zeros(size,2);
C2 = zeros(size,1);
Dist = zeros(size,1);
pos = struct('x', zeros(size,1), 'y', zeros(size,1));

% Generate circle (counter-clockwise from right side)
theta = linspace(0, 2*pi, size);
for i = 1:size
    pos.x(i) = R*cos(theta(i));  % x coordinate
    pos.y(i) = R*sin(theta(i));  % y coordinate
    C2(i) = 1/R;                 % Radius of curvature (constant)
    Dist(i) = theta(i)*R;
end

% Create track struct
C2 = C2.';
dist = Dist.';
save("Skidpad.mat",'pos','C2','dist');


%% generate acceleration track
L = 75;    % total length
size = 400;

% Initialize arrays
Pos = zeros(size,2);
C2 = zeros(size,1);
Dist = zeros(size,1);
pos = struct('x', zeros(size,1), 'y', zeros(size,1));

% Generate straight line
x = linspace(0, L, size);
for i = 1:size
    pos.x(i) = x(i);  % x coordinate
    pos.y(i) = 0;  % y coordinate
    C2(i) = 0;     % Radius of curvature (constant)
    Dist(i) = x(i);
end

% Create track struct
C2 = C2.';
dist = Dist.';
save("Acceleration.mat",'pos','C2','dist');