%run this program to obtain lap time simulation 
close all
clc

%open LTS directory
cd('D:\Patrick\VD SIM\LTS25')


%open track model file
cd('D:\Patrick\VD SIM\LTS25\Track Model')

track = 'Test Run 12.1 Track'; % 24 Autocross 24 Endurance Fastest
load(track)

%open tire model file

cd ('D:\Patrick\VD SIM\LTS25\Tyre Model')
tyre = 'R25B_V2';
load(tyre)

cd('D:\Patrick\VD SIM\LTS25\Vehicle Model')

%enter parameters of car
mass = 276;
air_density = 1.293;
frontel_area = 1.119492;
coef_drag = 1.619549;
coef_lift = 4.224138;%positive means downforce here
tyre_model = fit10psi; %select tyre with desired pressure
camber = 0;
wheelbase = 1.531;
maxsteer = 2.53073; %maximum steering angle in radian
%calculate max accel the car can give
%max_speed = sqrt((coef_friction*mass*9.81)/(0.5*air_density*frontel_area*(coef_drag-coef_friction*coef_lift)));

max_speed = 31.141; %change when powertrain comes in

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rolling Start: 1  Static Start: 0
rollingstart = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dist1 = dist;
C21 = C2;
pos1 = pos;

if rollingstart == 1
    len=length(C2);
    C2 = horzcat(C2,C2,C2);
    pos = horzcat(pos,pos,pos);
    dist2 = dist1+2*dist(len)-dist(len-1);
    dist3 = dist2+2*dist(len)-dist(len-1);
    dist = horzcat(dist,dist2,dist3);    
end

%generate boundary speed profile
BSP = bsp(mass,pos,C2,air_density,frontel_area,coef_lift,max_speed,tyre_model,camber);%output a nx2 array
%plot(BSP);%plot bsp diagram
%estimate slip angle base on track 
slip_ang = slip_angle(C2);
%generate limit speed profile with acceleration
%Accel_LSP = accel_lsp(dist,C2,BSP,mass,air_density,frontel_area,coef_lift,coef_drag,coef_friction);
Accel_LSP = accel_lsp_v2(dist,C2,pos,BSP,mass,air_density,frontel_area,coef_lift,coef_drag,camber,tyre_model);
%plot(Accel_LSP)
temp_lsp = lsp(C2,dist,camber,tyre_model,mass,air_density,frontel_area,coef_drag,Accel_LSP);
%
[final_lsp,yaw_diagram] = yawcal(temp_lsp,C2,maxsteer,wheelbase);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if rollingstart == 1
    len=length(C2);
    BSP = BSP(len/3:len*2/3-1);
    Accel_LSP = Accel_LSP(len/3:len*2/3-1);
    final_lsp = final_lsp(len/3:len*2/3-1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%return lap time simulation, lapsetime: lap time at each data point
[lap_time_sim,lapsetime] = Lap_Time_Simulation(final_lsp,dist1);

%return slip angle estimation
%alpha_profile = slip_model(final_lsp,slip_ang,mass,air_density,frontel_area,coef_lift,camber,tyre_model,C2);
%disp(alpha_profile);
Long_Accel = longG(final_lsp,dist1); %return Long G diagram
Lat_Accel = latG(final_lsp,C2); %return Lat G diagram
throttle_graph = throttle(final_lsp,dist1);
brake_graph = brake(final_lsp,dist,mass,air_density,frontel_area,coef_lift,camber,C2,tyre_model);


figure('Position',[625,60,900,700])
plotclr(pos1.x,-pos1.y,final_lsp*3.6);

figure
plot(lapsetime,final_lsp*3.6);
%hold on
%plot(dist1,BSP*3.6);
%figure
%plot(dist1,Long_Accel);
%figure
%plot(dist1,Lat_Accel);
disp(lap_time_sim);

%figure
%plot(dist,yaw_diagram);