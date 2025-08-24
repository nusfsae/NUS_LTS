%% Check MF5.2 Coefficients
clear; clc;

% Nominal Conditions
settings.Tyre.p0                     = 84000        ;     
settings.Tyre.Fz0 = 1080; % Nominal Load in [N]              

% Scaling Coefficients
settings.Tyre.LFZO                         = 1;  
settings.Tyre.LGAY                         = 1;
settings.Tyre.LCX                          = 1;          
settings.Tyre.LMUX                         = 1;          
settings.Tyre.LEX                          = 1;
settings.Tyre.LKX                          = 1;
settings.Tyre.LHX                          = 1;
settings.Tyre.LVX                          = 1;
settings.Tyre.LCY                          = 1;
settings.Tyre.LGX                          = 1;
settings.Tyre.LMUY                         = 1;
settings.Tyre.LEY                          = 1;
settings.Tyre.LKY                          = 1;
settings.Tyre.LHY                          = 1;
settings.Tyre.LVY                          = 1;
settings.Tyre.LTR                          = 1;
settings.Tyre.LRES                         = 1;
settings.Tyre.LXAL                         = 1;
settings.Tyre.LYKA                         = 1;
settings.Tyre.LVYKA                        = 1;
settings.Tyre.LS                           = 1;
settings.Tyre.LKYC                         = 1;
settings.Tyre.LKZC                         = 1;
settings.Tyre.LVMX                         = 1;
settings.Tyre.LMX                          = 1;
settings.Tyre.LMY                          = 1;
settings.Tyre.LMP                          = 1;

% longitudinal coefficients
settings.Tyre.PCX1                         = 1.5               ;
settings.Tyre.PDX1                         = 2.4722            ;
settings.Tyre.PDX2                         = -0.78691          ;
settings.Tyre.PDX3                         = 15                ;
settings.Tyre.PEX1                         = -2.5811e-13       ;
settings.Tyre.PEX2                         = -0.87477          ;
settings.Tyre.PEX3                         = -0.6              ;
settings.Tyre.PEX4                         = 0.9               ;
settings.Tyre.PKX1                         = 42.8193           ;
settings.Tyre.PKX2                         = -0.0001749        ;
settings.Tyre.PKX3                         = -0.49011          ;
settings.Tyre.PHX1                         = 0.00093775        ;
settings.Tyre.PHX2                         = -0.0013228        ;
settings.Tyre.PVX1                         = -0.02779          ;
settings.Tyre.PVX2                         = 0.089387          ;
settings.Tyre.PPX1                         = -1.0177           ;
settings.Tyre.PPX2                         = -1.3151           ;
settings.Tyre.PPX3                         = -0.2709           ;
settings.Tyre.PPX4                         = 0.81854           ;
settings.Tyre.RBX1                         = 5                 ;
settings.Tyre.RBX2                         = 5                 ;
settings.Tyre.RBX3                         = 0                 ;
settings.Tyre.RCX1                         = 1                 ;
settings.Tyre.REX1                         = -1                ;
settings.Tyre.REX2                         = -0.1              ;
settings.Tyre.RHX1                         = 0                 ;

% Lateral Coefficients
settings.Tyre.PCY1                         = 1.5               ;
settings.Tyre.PDY1                         = 2.5764            ;
settings.Tyre.PDY2                         = -0.47966          ;
settings.Tyre.PDY3                         = 1.2505            ;
settings.Tyre.PEY1                         = 0.44562           ;
settings.Tyre.PEY2                         = -0.15927          ;
settings.Tyre.PEY3                         = 0.055312          ;
settings.Tyre.PEY4                         = 11.0271           ;
settings.Tyre.PEY5                         = 166.589           ;
settings.Tyre.PKY1                         = -34.4974          ;
settings.Tyre.PKY2                         = 1.369             ;
settings.Tyre.PKY3                         = 0.60632           ;
settings.Tyre.PKY4                         = 2                 ;
settings.Tyre.PKY5                         = 83.7446           ;
settings.Tyre.PKY6                         = -4.1081           ;
settings.Tyre.PKY7                         = -0.79828          ;
settings.Tyre.PHY1                         = 0.0036805         ;
settings.Tyre.PHY2                         = 0.0016442         ;
settings.Tyre.PHY3                         = 0.1416            ;           % Missing from .Tir file, filled from heuristics
settings.Tyre.PVY1                         = 0.067909          ;
settings.Tyre.PVY2                         = 0.016368          ;
settings.Tyre.PVY3                         = 0.52062           ;
settings.Tyre.PVY4                         = -3.3053           ;
settings.Tyre.PPY1                         = 0.42092           ;
settings.Tyre.PPY2                         = 1.1945            ;
settings.Tyre.PPY3                         = -0.33642          ;
settings.Tyre.PPY4                         = -0.52307          ;
settings.Tyre.PPY5                         = -1.0699           ;
settings.Tyre.RBY1                         = 5                 ;
settings.Tyre.RBY2                         = 2                 ;
settings.Tyre.RBY3                         = 0.02              ;
settings.Tyre.RBY4                         = 0                 ;
settings.Tyre.RCY1                         = 1                 ;
settings.Tyre.REY1                         = -0.1              ;
settings.Tyre.REY2                         = 0.1               ;
settings.Tyre.RHY1                         = 0                 ;
settings.Tyre.RHY2                         = 0                 ;
settings.Tyre.RVY1                         = 0                 ;
settings.Tyre.RVY2                         = 0                 ;
settings.Tyre.RVY3                         = 0                 ;
settings.Tyre.RVY4                         = 0                 ;
settings.Tyre.RVY5                         = 0                 ;
settings.Tyre.RVY6                         = 0                 ;

% Slip Angle and Slip Ratio
sa_range = deg2rad(linspace(-15, 15, 100));
sx_range = linspace(-0.15, 0.15, 100);
fz_range = linspace(400, 1500, 4);

% Pure Lateral Force
for i = 1:numel(sa_range)
    outputs = MF52(0, sa_range(i), fz_range(2), 0, settings);
    fy(i) = outputs.FY;
    fx(i) = outputs.FX;
end

figure(1); tiledlayout(1,2);
nexttile
plot(rad2deg(sa_range), fy)

nexttile
plot(sx_range, fx)

% Pure Longitudinal Force
clear fy fx;

for i = 1:numel(sx_range)
    outputs = MF52(sx_range(i), 0, fz_range(2), 0, settings);
    fy(i) = outputs.FY;
    fx(i) = outputs.FX;
end

figure(2); tiledlayout(1,2);
nexttile
plot(sa_range, fy)

nexttile
plot(sx_range, fx)

%% Combined Slip
clear fy fx;
sa_range = deg2rad(linspace(-15, 15, 100));
sx_range = linspace(-0.15, 0.15, 100);

[X, Y] = meshgrid(sa_range, sx_range);
sa_range = X(:);
sx_range = Y(:);

for i = 1:numel(sx_range)
    outputs = MF52(sx_range(i), sa_range(i), fz_range(2), 0, settings);
    fy(i) = outputs.FY;
    fx(i) = outputs.FX;
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
