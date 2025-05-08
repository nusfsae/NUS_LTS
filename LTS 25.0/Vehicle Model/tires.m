% Reformat Tire model function to clean up main function
% All angles in degrees


function [Lat,Long] = tires(tyre_model,Fz,SR,SA,IA,P,V) 
SA = deg2rad(SA);
P = convpres(P, 'psi', 'Pa');
IA = deg2rad(IA);
phit = 0;
useMode = 121;
inputsMF = [Fz SR SA IA phit V P];

[ outMF ] = mfeval(tyre_model, inputsMF, useMode);
Long = 0.5904*Fz;
Lat = abs(outMF(2));

end

% long tire coeff = 0.5904