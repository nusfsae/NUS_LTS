cd ('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
HoosierR20 = mfeval.readTIR('HoosierR20.TIR'); %this tire no longitudinal data
HoosierLC0 = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')
tc_lat = 0.6077;                           tc_long = 0.6077;   
mass = 273;
tyre_model = HoosierLC0;
P = 10;   
P = convpres(P, 'psi', 'Pa');
alpha = 0;
long_slip = 0.15;
V = 5;   
camber = 0;
useMode = 121;
phit = 0;

Reaction_f = 0.25*(mass*9.81);
inputsMF = [Reaction_f long_slip alpha camber phit V P];

 [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
 Long = abs(outMF(1));
    Long = tc_long*Long;
R_wheel = 0.2032;
T_mu = Long*R_wheel;
    disp(T_mu);