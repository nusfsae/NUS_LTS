parameterSource = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');

Fz = 3200/4; %per tire
kappa = 0.1;
alpha = 0;
IA = 0;
phit = 0;
V = 10;
P = 68947.6;


alpha = deg2rad(alpha);

inputsMF = [Fz kappa alpha IA phit V P];

useMode = 121;

[ outMF ] = mfeval(parameterSource, inputsMF, useMode);

Fx = abs(outMF(1));
Fy = abs(outMF(2));

cd ('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
tyre = 'R25B_V2';
load(tyre)

tyre_model = fit10psi;

[Lat,Long,~] = tyres(IA,rad2deg(alpha),Fz,tyre_model); 

fprintf('Old model Long ');
disp(Long);
fprintf('New model Fx ');
disp(Fx);

fprintf('Old model Lat ');
disp(Lat);
fprintf('New model Fy');
disp(Fy);

