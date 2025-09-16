function settings = Hoosier16x6(settings)

settings.Tyre.FZ0 = 1100;
settings.Tyre.LFZO = 1.2;
% Scaling Factors

settings.Tyre.LGAY    =   1;
settings.Tyre.LHY     =   1;
settings.Tyre.LVY     =   1;
settings.Tyre.LCY     =   1;
settings.Tyre.LEY     =   1;
settings.Tyre.LHX     =   1;
settings.Tyre.LVX     =   1;
settings.Tyre.LGX     =   1;
settings.Tyre.LCX     =   1;
settings.Tyre.LEX     =   1;
settings.Tyre.LXAL    =   1;

settings.Tyre.LKY     =   1; % 1
settings.Tyre.LKX     =   1; %  0.7
settings.Tyre.LMUY    =   1; %0.5; % 0.38 Changed to Fit
settings.Tyre.LMUX    =   1; % 0.5; % 0.25 Changed to Fit

% Longitudinal Coefficients
settings.Tyre.PCX1   =   1.2602;
settings.Tyre.PDX1   =   2.354;
settings.Tyre.PDX2   =   -0.015401;
settings.Tyre.PDX3   =   -0.76992;
settings.Tyre.PEX1   =  -1.0845;
settings.Tyre.PEX2   =   2.3203;
settings.Tyre.PEX3   =   3.2136;
settings.Tyre.PEX4   =   -1.7027;
settings.Tyre.PKX1   =   39.334;
settings.Tyre.PKX2   =   -0.37146;
settings.Tyre.PKX3   =   0.37752;
settings.Tyre.PHX1   =   0.025058;
settings.Tyre.PHX2   =   -0.038843;
settings.Tyre.PVX1   =   -0.00045953;
settings.Tyre.PVX2   =   0.0013401;

% Combined Longitudinal Coefficients
settings.Tyre.RBX1   =   7.4574;
settings.Tyre.RBX2   =   -8.8044;
settings.Tyre.RCX1   =   1.5974;
settings.Tyre.REX1   =   0.22918;
settings.Tyre.REX2   =   -0.5217;
settings.Tyre.RHX1   =   0;

% Lateral Coefficients
settings.Tyre.PCY1   =   1.4;
settings.Tyre.PDY1   =   2.4;
settings.Tyre.PDY2   =   -0.4507889;
settings.Tyre.PDY3   =   20;
settings.Tyre.PEY1   =   0.01;
settings.Tyre.PEY2   =   0.05;
settings.Tyre.PEY3   =   10;
settings.Tyre.PEY4   =   0;
settings.Tyre.PKY1   =   -27.3678;
settings.Tyre.PKY2   =   1.242483;
settings.Tyre.PKY3   =   3;
settings.Tyre.PHY1   =   -0.00002845241;
settings.Tyre.PHY2   =   -0.0000329537;
settings.Tyre.PHY3   =   0.1416031;
settings.Tyre.PVY1   =   0;
settings.Tyre.PVY2   =   -0.009009;
settings.Tyre.PVY3   =   -0.5;
settings.Tyre.PVY4   =   -1;

% Combined Lateral Coefficients
settings.Tyre.RBY1   =   26.3099;
settings.Tyre.RBY2   =   20.3304;
settings.Tyre.RBY3   =   -0.015204;
settings.Tyre.RCY1   =   0.96889;
settings.Tyre.REY1   =   0.53522;
settings.Tyre.REY2   =   0.69602;
settings.Tyre.RHY1   =   0;
settings.Tyre.RHY2   =   0;
settings.Tyre.RVY1   =   0;
settings.Tyre.RVY2   =   0;
settings.Tyre.RVY3   =   0;
settings.Tyre.RVY4   =   0;
settings.Tyre.RVY5   =   0;
settings.Tyre.RVY6   =   0;

end