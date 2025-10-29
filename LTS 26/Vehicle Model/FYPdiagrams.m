% FYP Diagrams

%% GG Diagram
GGV_ss;
title('G-G Diagram at 15 m/s')
xlabel('ay (m/s^2)')
ylabel('ax (m/s^2)')
grid on
hold on; plot(-GG.ay, GG.ax, '-y', 'LineWidth', 5)
hold on; plot(GG.ay, GG.ax, '-y', 'LineWidth', 5)
fontsize(16,'points')
hold on
xline(0, 'w', 'LineWidth', 1.5);  % Vertical axis
yline(0, 'w', 'LineWidth', 1.5);  % Horizontal axis

%% half GG diagram
figure
title('G-G Diagram at 15 m/s')
xlabel('ay (m/s^2)')
ylabel('ax (m/s^2)')
grid on
hold on; plot(GG.ay, GG.ax, '-y', 'LineWidth', 5)
fontsize(16,'points')
hold on
xline(0, 'w', 'LineWidth', 1.5);  % Vertical axis
yline(0, 'w', 'LineWidth', 1.5);  % Horizontal axis

%% GG diagram with different speed
figure
for V = [12,15,25]
    GGV_ss; 
    plot(GG.ay, GG.ax,'DisplayName',"Speed = "+ V+"m/s ", 'LineWidth', 2); 
    hold on
    legend
end
title('G-G Diagram at different speed')
xlabel('ay (m/s^2)')
ylabel('ax (m/s^2)')
grid on
hold on; 
fontsize(16,'points')
xlim([0 25])

%% GGV plot
figure

% First, check if all speeds have the same number of points
num_speeds = numel(velocityRange);
num_points = numel(GG.speed(1).ay);
% Preallocate matrices
ay_matrix = zeros(num_speeds, num_points);
ax_matrix = zeros(num_speeds, num_points);
speed_matrix = zeros(num_speeds, num_points);
% Fill matrices
for i = 1:num_speeds
    GG.speed(i).aymax = max(GG.speed(i).ay);    
    % Make sure data is a row vector
    ay_matrix(i, :) = GG.speed(i).ay(:)';  % Force row vector with (:)'
    ax_matrix(i, :) = GG.speed(i).ax(:)';
    speed_matrix(i, :) = GG.speed(i).speed;
end

% Create surface plot
surf(ay_matrix, ax_matrix, speed_matrix)
xlabel('Lateral Acceleration ay (m/s²)')
ylabel('Longitudinal Acceleration ax (m/s²)')
zlabel('Speed (m/s)')
title('GG Diagram - 3D Performance Envelope')
view(3)
grid on

%% V vs Radius
% Extract data
speed = performance.speed;
radius = performance.radius;
% Define a smooth radius range
radius_query = linspace(0, 30, 200);
% Evaluate the spline at those radii
speed_interp = ppval(PerfEnv, radius_query);
% Plot
figure
plot(radius_query, speed_interp, 'LineWidth', 2);
xlabel('Cornering Radius (m)');
ylabel('Speed (m/s)');
title('Performance Envelope: Speed vs Cornering Radius');
grid on
fontsize(16,'points')
%% V vs Curvature
figure
plot(1./radius_query,speed_interp, 'LineWidth', 2);
xlabel('Curvature (1/m)');
ylabel('Speed (m/s)');
title('Performance Envelope: Speed vs Curvature');
grid on
fontsize(16,'points')

%% track curvature
figure
tiledlayout(2,1);
nexttile
plot(dist,abs(C2),'c','LineWidth', 2);
xlabel('Accumulated Distance (m)');
ylabel('Radius of Curvature (1/m)');
title('Track Model: Curvature');
grid on
fontsize(13,'points')

nexttile
for i = 1:length(C2)  
    radius = abs(1/C2(i));
    % if radius larger than 30m, assume straight
    if radius > 30      
        sim.speed(i,1) = v_max;
        sim.latG(i,1) = 0;   
    % cornering scenario
    else 
        vy = ppval(PerfEnv,radius);
        sim.speed(i,1) = vy;  
    end
end
plot(dist,sim.speed,'c','LineWidth', 2);
title('Boundary Speed Profile');
ylabel('Cornering Speed (m/s)');
xlabel('Accumulated Distance (m)');
ylim([0 40])
grid on
fontsize(13,'points')

%% complete speed profile
figure
% corner
for i = 1:length(C2)  
    radius = abs(1/C2(i));
    % if radius larger than 30m, assume straight
    if radius > 30      
        sim.speed(i,1) = v_max;
        sim.latG(i,1) = 0;   
    % cornering scenario
    else 
        vy = ppval(PerfEnv,radius);
        sim.speed(i,1) = vy;  
    end
end
plot(dist,sim.speed,'c','LineWidth', 1.5,'DisplayName', 'Boundary Speed Profile');
ylabel('Speed (m/s)');
xlabel('Accumulated Distance (m)');
ylim([0 40])
grid on
fontsize(13,'points')
% accelerate
for i = 1:length(C2)-1
    % turn radius of current point
    radius = abs(1/C2(i));
    % distance to the next point
    distance = dist(i+1)-dist(i);
    % current speed and lateral acceleration
    v0 = sim.speed(i);
    ay = v0^2/radius;
    % available longitudinal acceleraton
    ax = axAccel(v0,ay);
    % calculate accelerated speed
    v = sqrt(v0^2+2*ax*distance);
    if v<=sim.speed(i+1)
        sim.speed(i+1)=v;
    end
end
% brake
for i = length(C2):-1:2
    % turn radius of current point
    radius = abs(1/C2(i));
    % distance to the next point
    distance = dist(i)-dist(i-1);
    % current speed and lateral acceleration
    v0 = sim.speed(i);
    ay = v0^2/radius;
    % available longitudinal deceleraton
    ax = axBrake(v0,ay);
    v = sqrt(v0^2-2*ax*distance);  % take note ax is a negative value
    % replace speed value if below boundary
    if v<=sim.speed(i-1)
        sim.speed(i-1)=v;
    end
end
hold on
plot(dist,sim.speed,'r','LineWidth', 1.5,'DisplayName', 'Limit Speed Profile');
title('Speed Profile');
ylabel('Speed (m/s)');
xlabel('Accumulated Distance (m)');
ylim([0 40])
grid on
fontsize(13,'points')