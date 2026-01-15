% [Fx, Fy] = MF61(Fz, kappa, alpha, gamma, Vcx, pres, tirParams)
% Fz (N)
% kappa (unitless)
% alpha (rad)
% gamma (rad)
% Vcx (m/s)
% pres (Pa)

cd("D:\Mfeval\MFeval")
% add filepath of TIR file
para = mfeval.readTIR("D:\Tire Data and Model\TTC Round 6\43105_18x7.5_10_R25B_7.tir");

function Fx = Fx(Fz, kappa, alpha, gamma, Vcx, pres, tirParams)
    [Fx,~] = SM_MF61(Fz, kappa, alpha, gamma, Vcx, pres, tirParams);
end

function Fy = Fy(Fz, kappa, alpha, gamma, Vcx, pres, tirParams)
    [~,Fy] = SM_MF61(Fz, kappa, alpha, gamma, Vcx, pres, tirParams);
end

%% surface plot for Force vs Fz
gamma = 0;
Vcx = 10;
pres = 70000;

% Fx
figure
alpha = 0; % Change to see the pure/combined force output
fsurf(@(kappa, Fz) Fx(Fz, kappa, alpha, gamma, Vcx, pres, para), [-0.8 0.8 500 2000]);
title('Fx vs Fz')
xlabel('slip ratio')
ylabel('normal load')
zlabel('Fx')

% Fx
% figure
% alpha = 0; % Change to see the pure/combined force output
% fsurf(@(kappa, Fz) Fx(Fz, kappa, alpha, gamma, Vcx, pres, para)/Fz, [-0.8 0.8 500 700]);
% title('Accel vs Fz')
% xlabel('slip ratio')
% ylabel('normal load')
% zlabel('Fx')

% Fy
figure
kappa = 0; % Change to see the pure/combined force output
fsurf(@(alpha, Fz) Fy(Fz, kappa, alpha, gamma, Vcx, pres, para), [-1.5 1.5 500 700]);
title('Fy vs Fz')
xlabel('slip angle')
ylabel('normal load')
zlabel('Fy')

%% surface plot for Force vs Vcx
gamma = 0;
Fz = 66*9.81;
pres = 62052.8;

% Fx
figure

alpha = 0;
fsurf(@(kappa, Vcx) Fx(Fz, kappa, alpha, gamma, Vcx, pres, para), ...
      [-0.5 0.5 0.1 100]);

title('Fx vs Vcx')
xlabel('slip ratio')
ylabel('Velocity')
zlabel('Fx')

hold on

% your line
x_coords = [0.133, 0.133];
y_coords = [0.1,   100];
z_coords = [1.9922e3, 1.9922e3];

plot3(x_coords, y_coords, z_coords, 'r-', 'LineWidth', 2)

hold off


% Fy
figure
kappa = 0; % Change to see the pure/combined force output
fsurf(@(alpha, Vcx) Fy(Fz, kappa, alpha, gamma, Vcx, pres, para),...
    [-0.5 0.5 0.1 25]);
title('Fy vs Vcx')
xlabel('slip angle')
ylabel('Velocity')
zlabel('Fy')
%% surface plot for Force vs slip
gamma = 0;
Fz = 600;
pres = 83000;
Vcx = 10;

load('B1654run35.mat', 'FX', 'FY', 'FZ', 'IA', 'SA', 'SL', 'P')
dataClean = (IA <0.5) & (IA > -0.5) & (FZ < -550) & (FZ > -650) & (P > 80) & (P < 85);
FX_clean = FX(dataClean);
FY_clean = FY(dataClean);
SA_clean = tan(SA(dataClean)*2*pi/360);
SL_clean = SL(dataClean);

% Fx
figure
fsurf(@(kappa, alpha) Fx(Fz, kappa, alpha, gamma, Vcx, pres, para), [-0.5 0.5 -0.5 0.5]);
title('Fx vs slip ratio')
xlabel('slip ratio')
ylabel('slip angle')
zlabel('Fx combined')
hold on;
plot3(SL_clean,SA_clean,FX_clean,'o',Color='r')
hold off;

% Fy
figure
fsurf(@(kappa, alpha) Fy(Fz, kappa, alpha, gamma, Vcx, pres, para), [-0.5 0.5 -0.5 0.5]);
title('Fy vs slip angle')
xlabel('slip ratio')
ylabel('slip angle')
zlabel('Fy combined')
hold on;
plot3(SL_clean,SA_clean,FY_clean,'o',Color='r')
hold off;


%% Friction Circle at Different Normal Loads
gamma = 0;
pres = 83000;
Fz = 800;
Vcx = 10;
long = [];
lat = [];

for SR = -0.25:0.005:0.25
    for SA = -0.25:0.005:0.25
        [fx, fy] = SM_MF61(Fz, SR, SA, gamma, Vcx, pres, para);
        long = horzcat(long, fx);
        lat = horzcat(lat, fy);
    end
end


gamma = 0;
pres = 83000;
Fz = 600;
Vcx = 10;
long1 = [];
lat1 = [];

for SR = -0.25:0.005:0.25
    for SA = -0.25:0.005:0.25
        [fx, fy] = SM_MF61(Fz, SR, SA, gamma, Vcx, pres, para);
        long1 = horzcat(long1, fx);
        lat1 = horzcat(lat1, fy);
    end
end

gamma = 0;
pres = 83000;
Fz = 1000;
Vcx = 10;
long2 = [];
lat2 = [];

for SR = -0.25:0.005:0.25
    for SA = -0.25:0.005:0.25
        [fx, fy] = SM_MF61(Fz, SR, SA, gamma, Vcx, pres, para);
        long2 = horzcat(long2, fx);
        lat2 = horzcat(lat2, fy);
    end
end

figure
plot(long,lat, '*', Color = 'w')
hold on;
plot(long1, lat1, '.', Color='', MarkerSize=8);
hold on;
plot(long2, lat2, '*', Color='#CCFFCC', MarkerSize=4)
hold off;

%% Friction Circle with TTC Data
gamma = 0;
Fz = 590;
pres = 83000;
Vcx = 10;
long = [];
lat = [];

load('B1654run35.mat', 'FX', 'FY', 'FZ', 'IA', 'SA', 'SL', 'P')
dataClean = (IA <0.5) & (IA > -0.5) & (FZ < -550) & (FZ > -650) & (P > 80) & (P < 85);
FX_clean = FX(dataClean);
FY_clean = FY(dataClean);
SA_clean = tan(SA(dataClean)*2*pi/360);
SL_clean = SL(dataClean);

for SR = -0.1650:0.005:0.1440
    for SA = -0.1065:0.005:0
        [fx, fy] = SM_MF61(Fz, SR, SA, gamma, Vcx, pres, para);
        long = horzcat(long, fx);
        lat = horzcat(lat, fy);
    end
end

figure
plot(long, lat, '.')
hold on;
plot(FX_clean,FY_clean,'.')
hold off;