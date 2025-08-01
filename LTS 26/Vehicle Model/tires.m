% Reformat Tire model function to clean up main function
% Fz: Normal Load (N)
% SR: Slip Ratio (unitless)
% SA: Slip Angle (rad)
% IA: Inclination Angle (rad)
% P:  Tyre Pressure (Psi)
% V:  Vehicle speed (m/s)

function [Lat,Long] = tires(tyre_model,Fz,SR,SA,IA,P,V)

P = convpres(P, 'psi', 'Pa');
phit = 0;
useMode = 121;
inputsMF = [Fz SR SA IA phit V P];

[ outMF ] = mfeval(tyre_model, inputsMF, useMode);


Lat = abs(outMF(2));
Long = abs(outMF(1));

end

% long tire coeff = 1.1808