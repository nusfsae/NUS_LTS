R25B =  readTIR("D:\TTC Round 8\RunData Cornering Matlab SI 10inch round8\16x7.5-10x7R25B\16x7.5-10x7R25B.tir");
R20 = readTIR("D:\TTC Round 9\RunData_Cornering_MATLAB_SI_Round9\16x7.5-10x7R20\16x17.5-10x7R20.tir");

slipAngles = linspace(-10, 10, 100); % degrees
Fz = 3000;       % in Newtons
P = 220000;      % inflation pressure in Pa
Vx = 30;         % longitudinal speed in m/s

% Initialize output arrays
Fy1 = zeros(size(slipAngles));
Fy2 = zeros(size(slipAngles));
