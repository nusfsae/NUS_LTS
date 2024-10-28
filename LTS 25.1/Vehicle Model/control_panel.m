%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Run this program to obtain simulation results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
clc

%open LTS directory
cd('C:\Users\Patri\OneDrive - National University of Singapore\Documents\NUS\Formula SAE\LTS25')


%open track model file
cd('C:\Users\Patri\OneDrive - National University of Singapore\Documents\NUS\Formula SAE\LTS25\Track Model')

track = '241013 JTC PM'; % 24 Autocross % 24 Endurance Fastest % 241013 JTC PM % Skidpad_10m
load(track)

%open tire model file

cd ('C:\Users\Patri\OneDrive - National University of Singapore\Documents\NUS\Formula SAE\LTS25\Tyre Model')
tyre = 'R25B_V2';
load(tyre)

cd('C:\Users\Patri\OneDrive - National University of Singapore\Documents\NUS\Formula SAE\LTS25\Vehicle Model')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Enter parameters of car %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Vehicle Mass (kg)
mass = 274; 

% Air Density (kg/m^3)
air_density = 1.196;

% Car Frontel Area (m^2)
frontel_area = 1.157757;

% Coefficient of Drag
CDc = 1.469746;
CDs = 1.619549;

% Coefficient of Lift
CLc = 3.469153;
CLs = 4.224138;

% Tyre Model 
tyre_model = fit10psi;

% Static Camber (Radian)
camber = 0;

% Vehicle Wheelbase (m)
wheelbase = 1.531;

% Maximum Steering Angle (Degree)
maxsteer = 32.372; 

% Maximum Vehicle Speed (m/s)
max_speed = 31.141; 

% Rolling Start: 1  Static Start: 0
rollingstart = 0;

% Turn On: 1  Turn Off: 0  Temporary Off: 2
masterswitch = 0;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DO NOT CHANGE ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^%

dist1 = dist;
C21 = C2;
%pos1 = pos;

if rollingstart == 1
    len=length(C2);
    C2 = horzcat(C2,C2,C2);
    pos = horzcat(pos,pos,pos);
    dist2 = dist1+2*dist(len)-dist(len-1);
    dist3 = dist2+2*dist(len)-dist(len-1);
    dist = horzcat(dist,dist2,dist3);    
end

%generate boundary speed profile
BSP = bsp(mass,C2,air_density,frontel_area,CLc,max_speed,tyre_model,camber,maxsteer);%output a nx2 array
%plot(BSP);%plot bsp diagram
%estimate slip angle base on track 
slip_ang = slip_angle(C2);
%generate limit speed profile with acceleration
%Accel_LSP = accel_lsp(dist,C2,BSP,mass,air_density,frontel_area,coef_lift,coef_drag,coef_friction);
Accel_LSP = accel_lsp_v2(dist,C2,BSP,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,tyre_model);
%plot(Accel_LSP)
temp_lsp = lsp(C2,dist,camber,tyre_model,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,Accel_LSP);
%
[final_lsp,yaw_diagram] = yawcal(temp_lsp,C2,maxsteer,wheelbase);

if rollingstart == 1
    len=length(C2);
    BSP = BSP(len/3:len*2/3-1);
    Accel_LSP = Accel_LSP(len/3:len*2/3-1);
    final_lsp = final_lsp(len/3:len*2/3-1);
end

%return lap time simulation, lapsetime: lap time at each data point
[lap_time_sim,lapsetime] = Lap_Time_Simulation(final_lsp,dist1);

%return slip angle estimation
%alpha_profile = slip_model(final_lsp,slip_ang,mass,air_density,frontel_area,coef_lift,camber,tyre_model,C2);
%disp(alpha_profile);
Long_Accel = longG(final_lsp,dist1); %return Long G diagram
Lat_Accel = latG(final_lsp,C2); %return Lat G diagram
throttle_graph = throttle(final_lsp,dist1);
brake_graph = brake(final_lsp,dist1,mass,air_density,frontel_area,CLs,CLc,CDs,CDc,camber,C2,tyre_model);


%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DO NOT CHANGE ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




if masterswitch == 1
    fprintf("Simulated Lap Time is "+lap_time_sim+" seconds"+".\n");
end


if masterswitch == 1
    if ~isempty(pos)
        figure('Position',[625,60,900,700])
        plotclr(pos1.x,-pos1.y,final_lsp*3.6);
        sgtitle("Speed Profile on Track Model")
    end
end


if masterswitch == 2
    figure
    plot(dist1,final_lsp*3.6);
    hold on
    plot(dist1,BSP*3.6);
    xlabel("Distance (m)")
    ylabel("Speed (km/h)")
    sgtitle("Speed Profile")
end


if masterswitch == 2
    figure
    plot(dist1,Long_Accel);
    xlabel("Distance (m)")
    ylabel("Acceleration (G)")
    sgtitle("Longitudinal Acceleration")
end


if masterswitch == 2
    figure
    plot(dist1,Lat_Accel);
    xlabel("Distance (m)")
    ylabel("Acceleration (G)")
    sgtitle("Lateral Acceleration")
end


if masterswitch == 2
    figure
    plot(dist1,throttle_graph)
    xlabel("Distance (m)")
    ylabel("Throttle Percentage")
    sgtitle("Throttle Graph")
end


if masterswitch == 1
    figure
    plot(dist1,brake_graph);
    xlabel("Distance (m)")
    ylabel("Brake Percentage")
    sgtitle("Brake Graph")
end


if masterswitch == 2
    figure
    plot(dist,yaw_diagram);
    xlabel("Distance (m)")
    ylabel("Yaw Rate")
    sgtitle("Yaw Rate Graph")
end