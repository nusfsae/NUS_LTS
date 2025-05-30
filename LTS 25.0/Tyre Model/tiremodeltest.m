cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
HoosierLC0 = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');
HoosierR20 = mfeval.readTIR('HoosierR20.TIR');
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')

tyre_model = HoosierR20;

% cd ('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')

Fz = 700; %per tire
SA = 0;
SR = 0;
IA = 0;
V = 10;
P = 9;
R_wheel = 0.2032;


[Lat,Long] = tires(tyre_model,Fz,SR,SA,IA,P,V);

disp(Long);

a = Long*0.6*4/(200*9.81);

disp(a);



function [Lat,Long] = tires(tyre_model,Fz,SR,SA,IA,P,V) 


tire2 = 'R25B_V2';
load(tire2)
[~,Long,~] = tyres(IA,SA,Fz,fit10psi);


SA = deg2rad(SA);
P = convpres(P, 'psi', 'Pa');
IA = deg2rad(IA);
phit = 0;
useMode = 121;
inputsMF = [Fz SR SA IA phit V P];

[ outMF ] = mfeval(tyre_model, inputsMF, useMode);
Lat = abs(outMF(2));



end


% Known variables
mass = 262;
air_density = 1.196;
frontel_area = 1.157757;
CLc = 3.782684;
camber = 0;
phit = 0;
P = 9; % convert psi to pa
useMode = 121;
max_speed = 40;
del_max = rad2deg(0.565);
curv = 8.374;
sen_lat = 1;
tc_long = 0.6077;
sen_long = 1;
max_rpm = 5500;
FDR = 3.36;
R = 0.2032;
sen_lat = 1;

Fx = 1000;
Weight = mass*9.81;
Downforce = 0.5*air_density*frontel_area*CLc*(76/3.6)^2;
Fz = Weight+Downforce;
Fz=Fz/4;
mu = Fx/Fz;
disp(mu)