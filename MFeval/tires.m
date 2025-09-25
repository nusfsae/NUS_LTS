% Reformat Tire model function to clean up main function
% All angles in degrees


function [Lat,Long] = tires(tyre_model,Fz,SR,SA,IA,V,P)


%{
    if SA<8
    Long = ((10-SA)/10)*1.1808*Fz;
else
    Long = 0;
end
SA = deg2rad(SA);
P = convpres(P, 'psi', 'Pa');
IA = deg2rad(IA);
%}

phit = 0;
useMode = 121;
inputsMF = [Fz SR SA IA phit V P];

[ outMF ] = mfeval(tyre_model, inputsMF, useMode);

Long = outMF(:,1);
Lat = outMF(:,2);

end

% long tire coeff = 1.1808