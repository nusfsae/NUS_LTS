
parameterSource = mfeval.readTIR('HoosierR20.TIR');

Fz = 2000;
kappa = 0;
alpha = 0.174533;
IA = 0;
phit = 0;
V = 0;
P = 68947.6;

inputsMF = [Fz kappa alpha IA phit V P];

useMode = 121;

[ outMF ] = mfeval(parameterSource, inputsMF, useMode);
Fy = abs(outMF(2));


% inputsMF
% Fz (N)
% Longitudinal slip kappa (-1 to 1)
% Side slip (rad) alpha
% IA (rad)
% turn slip phit (put 0)
% velocity (m/s)
% pressure (Pa) optional
% rotational speed (rad/s) optional

% useMode = 111

% outMF
% outMF(1) = Fx; outMF(2) = Fy; outMF(3) = Fz; 
% outMF(4) = Mx overturning moment; outMF(5) = My rolling resistance moment
% outMF(6) = Mz self aligning torque

%%%%%%%%%%%%%
nPoints = 200;

% Pure lateral test case
Fz      = ones(nPoints,1).*3000;            % vertical load         (N)
kappa	= ones(nPoints,1).*0;               % longitudinal slip 	(-) (-1 = locked wheel)
alpha	= linspace(-0.3,0.3, nPoints)';     % side slip angle    	(radians)
gamma	= ones(nPoints,1).*0;               % inclination angle 	(radians)
phit 	= ones(nPoints,1).*0;               % turnslip            	(1/m)
Vx   	= ones(nPoints,1).*16;              % forward velocity   	(m/s)

% Create a string with the name of the TIR file
TIRfile = 'MagicFormula61_Parameters.TIR';

% Select a Use Mode
useMode = 111;

% Wrap all inputs in one matrix
inputs = [Fz kappa alpha gamma phit Vx];

% Store the output from mfeval in a 2D Matrix
output = mfeval(TIRfile, inputs, useMode);



% Extract variables from output MFeval. For more info type "help mfeval"
Fy = output(:,2);
Mz = output(:,6);
Mx = output(:,4);
SA = rad2deg(output(:,8)); % Convert to degrees
t = output(:,16);

figure
subplot(2,2,1)
plot(SA, Fy)
grid on
title('Fy-SA')
xlabel('Slip Angle (deg)')
ylabel('Lateral Force (N)')

subplot(2,2,2)
plot(SA, Mz)
grid on
title('Mz-SA')
xlabel('Slip Angle (deg)')
ylabel('Self aligning moment (Nm)')

subplot(2,2,3)
plot(SA, t)
grid on
title('t-SA')
xlabel('Slip Angle (deg)')
ylabel('pneumatic trail (m)')

subplot(2,2,4)
plot(SA, Mx)
grid on
title('Mx-SA')
xlabel('Slip Angle (deg)')
ylabel('Overturning moment (Nm)')