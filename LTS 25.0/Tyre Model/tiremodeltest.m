parameterSource = mfeval.readTIR('HoosierR20.TIR');

Fz = 1000; %per tire
kappa = 0;
alpha = 0.174533;
IA = 0;
phit = 0;
V = 20;
P = 68947.6;

inputsMF = [Fz kappa alpha IA phit V P];

useMode = 121;

[ outMF ] = mfeval(parameterSource, inputsMF, useMode);
Fy = abs(outMF(2));

cd ('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
tyre = 'R25B_V2';
load(tyre)

tyre_model = fit10psi;

[Lat,~,~] = tyres(IA,rad2deg(alpha),Fz,tyre_model); 

fprintf('Old model ');
disp(Lat);
fprintf('New model ');
disp(Fy);

