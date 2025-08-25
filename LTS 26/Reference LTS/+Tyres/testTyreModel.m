%% Check MF5.2 Coefficients
clear; clc;

% Car Settings struct
settings = struct;

% Select Tyres
settings = Tyres.Hoosier16x75(settings);
% settings = Tyres.Hoosier16x6(settings);

% Slip Angle and Slip Ratio
sa_range = deg2rad(linspace(-15, 15, 100));
sx_range = linspace(-0.15, 0.15, 100);
fz_range = linspace(400, 1500, 4);

% Pure Lateral Force
for i = 1:numel(sa_range)
    [FY, FX] = Tyres.MF52(0, sa_range(i), fz_range(2), 0, settings);
    fy(i) = FY;
    fx(i) = FX;
end

figure(1); t = tiledlayout(1,2);
title(t, 'Pure Lateral Slip')
nexttile
plot(rad2deg(sa_range), fy)
xlabel('Slip Angle [deg]')
ylabel('Fy [N]')

nexttile
plot(zeros(numel(fx),1), fx)
xlabel('Slip Ratio [-]')
ylabel('Fx [N]')

% Pure Longitudinal Force
clear fy fx;

for i = 1:numel(sx_range)
    [FY, FX] = Tyres.MF52(sx_range(i), 0, fz_range(2), 0, settings);
    fy(i) = FY;
    fx(i) = FX;
end

figure(2); t = tiledlayout(1,2);
title(t,'Pure Longitudinal Slip')
nexttile
plot(zeros(numel(fy),1), fy)
xlabel('Slip Angle [deg]')
ylabel('Fy [N]')

nexttile
plot(sx_range, fx)
xlabel('Slip Ratio [-]')
ylabel('Fx [N]')

%% Combined Slip
clear fy fx;
sa_range = deg2rad(linspace(-15, 15, 100));
sx_range = linspace(-0.15, 0.15, 100);

[X, Y] = meshgrid(sa_range, sx_range);
sa_range = X(:);
sx_range = Y(:);

for i = 1:numel(sx_range)
    [FY, FX] = Tyres.MF52(sx_range(i), sa_range(i), fz_range(2), 0, settings);
    fy(i) = FY;
    fx(i) = FX;
end

figure(3); clf; t = tiledlayout(2,3);
title(t, 'Combined Slip')
nexttile
scatter(fy, fx)
xlabel('Fy [N]')
ylabel('Fx [N]')
axis equal
grid on

nexttile
scatter(rad2deg(sa_range), fy, [], sx_range, '.')
colorbar
ylabel('Fy [N]')
xlabel('Slip Angle [deg]')

nexttile
scatter(sx_range, fx, [], sa_range, '.')
colorbar
ylabel('Fx [N]')
xlabel('Slip Ratio [-]')

nexttile
title('Sx-Fy Dependency')
scatter(sx_range, fy, [], sa_range, '.')
colorbar
ylabel('Fy [N]')
xlabel('Slip Ratio [-]')

nexttile
title('Sa-Fx Dependency')
scatter(rad2deg(sa_range), fx, [], sx_range, '.')
colorbar
ylabel('Fx [N]')
xlabel('Slip Angle [deg]')
ylim([-3000 3000])

%% 
