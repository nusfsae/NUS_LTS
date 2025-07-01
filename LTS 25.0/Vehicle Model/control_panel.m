%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Run this program to obtain simulation results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warning('off','all')
warning


clc

%open LTS directory

cd('D:\FSAEMain\LTS 25.0')

%open track model file

cd('D:\FSAEMain\LTS 25.0\Track Model')
track = 'JTC 2025 v2.mat'; % 24 Autocross % 24 Endurance Fastest % 241013 JTC PM % Skidpad_10m %JTC 2025 v1.mat %JTC v2.mat
load(track)

%open tire model file

cd ('D:\FSAEMain\LTS 25.0\Tyre Model')
HoosierR20 = readTIR('HoosierR20.TIR'); %this tire no longitudinal data
HoosierLC0 = readTIR('Hoosier_6_18_10_LC0_C2000.TIR');



cd('D:\FSAEMain\LTS 25.0\Vehicle Model')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Enter settings of car %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Vehicle Mass (kg)                        % Air Density (kg/m^3)                   % Tyre Model
mass = 274;                                air_density = 1.196;                     tyre_model = HoosierLC0;

% Tyre Pressure (psi)                      % Longitudinal Slip                      % Turn Slip (1/m)
P = 9;                                     long_slip = 0.145;                       phit = 0;

% Coefficient of Drag (Straight)           % Coefficient of Lift (Straight)         % Coefficient of Lift (Corner)
CDs = 1.54709;                             CLs = 4.116061;                          CLc = 3.782684;
                   
% Coefficient of Drag (Corner)             % Vehicle Wheelbase (m)                  % Wheel Radius (m)
CDc = 1.410518;                            wheelbase = 1.531;                       R_wheel = 0.2032;

% Static Camber (Radian)                   % Car Frontel Area (m^2)                 % Maximum Steering Angle (Degree)
camber = 0;                                frontel_area = 1.157757;                 maxsteer = 32.372; 

% Maximum Motor Torque (Nm)                % Final Drive Ratio                      % Motor Maximum Rotation Speed (RPM)
max_torque = 169.58;                       FDR = 3.36;                              max_rpm = 5500;

% Lateral Tire Correlation Factor          % Longitudinal Tire Correlation Factor   % Longitudinal Tire Sensitivity 
tc_lat = 0.6077;                           tc_long = 1;                             sen_long = 1;

% Lateral Tire Sensitivity                 % Torque Setting
sen_lat = 1;                               Ipeak = 0.6;

% Tire Model Setting                       % CG Height                              % Aero Balance
useMode = 121;                             cg_h = 0.256;                            ab = 0.5310665;

%Accum Voltage                             %Kv Constant
Accum_Voltage = 333.75;                    Voltage_constant = 14;

%kW                                        %Conversion Factor
power_cap = 80;                            conversion_factor = 9550;

% Rolling Start: 1  Static Start: 0
rollingstart = 1;

% Turn On: 1  Turn Off: 0  Temporary Off: 2
masterswitch = 1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DO NOT CHANGE ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^%

tic

% % Estimation Round with single-wheel model

[Tractive_force, Speed] = TractiveForce(max_torque, max_rpm, conversion_factor, power_cap, Ipeak, FDR, R_wheel);

cd('D:\FSAEMain\LTS 25.0\One_wheel')

[BSP,Accel_LSP,Final_LSP,lap_time_sim,lapsetime,Long_Accel,Lat_Accel,throttle_graph,brake_graph,angleChange] = main1w( ...
    mass,C2,dist,pos,air_density,tyre_model,P,long_slip,phit,CLc,CLs,CDc,CDs,frontel_area,camber,maxsteer, ...
    max_rpm,max_torque,FDR,R_wheel,tc_lat,tc_long,sen_lat,sen_long,wheelbase,rollingstart,useMode,Ipeak, Tractive_force, Speed);


% % Precise Simulation Round with two-wheel model

cd('D:\FSAEMain\LTS 25.0\Vehicle Model')

[BSP,Accel_LSP,Final_LSP,lap_time_sim,lapsetime,Long_Accel,Lat_Accel,throttle_graph,brake_graph,torque_dist] = main( ...
    mass,C2,dist,pos,air_density,tyre_model,P,long_slip,phit,CLc,CLs,CDc,CDs,frontel_area,camber,maxsteer, ...
    max_rpm,max_torque,FDR,R_wheel,tc_lat,tc_long,sen_lat,sen_long,wheelbase,rollingstart,useMode,Ipeak,cg_h,Long_Accel,ab, angleChange, Tractive_force, Speed);

cd("D:\FSAEMain\LTS 25.0\Powertrain")
[kWh_Usage, power_map] = predictEnergyUsage(torque_dist, Final_LSP, throttle_graph, dist, lapsetime);

cd('D:\FSAEMain\LTS 25.0\Vehicle Model')

toc

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ DO NOT CHANGE ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if masterswitch == 1
    if ~isempty(pos)
        figure('Position',[625,60,900,700])
        plotclr(pos.x,pos.y,Final_LSP*3.6,'.');
        sgtitle("Speed Profile on Track Model")
    end
end


if masterswitch == 1
    figure
    plot(dist,Final_LSP*3.6);
    % hold on
    % plot(dist,BSP*3.6);
    xlabel("Distance (m)")
    ylabel("Speed (km/h)")
    sgtitle("Speed Profile")
end


if masterswitch == 1
    figure
    plot(dist,Long_Accel);
    xlabel("Distance (m)")
    ylabel("Acceleration (G)")
    sgtitle("Longitudinal Acceleration")
end


if masterswitch == 1
    figure
    plot(dist,Lat_Accel);
    xlabel("Distance (m)")
    ylabel("Acceleration (G)")
    sgtitle("Lateral Acceleration")
end


if masterswitch == 2
    figure
    plot(dist,throttle_graph)
    xlabel("Distance (m)")
    ylabel("Throttle Percentage (%)")
    sgtitle("Throttle Graph")
    ylim([0,120])
end


if masterswitch == 2
    figure
    plot(dist,brake_graph);
    xlabel("Distance (m)")
    ylabel("Brake Percentage (%)")
    sgtitle("Brake Graph")
end




