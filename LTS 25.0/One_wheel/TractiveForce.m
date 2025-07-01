% Revamped Powertrain Modelling
function [Tractive_force, Speed] = TractiveForce(max_torque, max_rpm, conversion_factor, power_cap, Ipeak, FDR, R_wheel)

rated_RPM = (power_cap * conversion_factor) / max_torque;
RPM_range = transpose(linspace(0,max_rpm,200));
torque = zeros(length(RPM_range),1);
rated_torque = Ipeak * max_torque ;

for i = 1: length(RPM_range)
    if RPM_range(i) < rated_RPM
        torque(i) = rated_torque ;
    else 
        torque(i) = rated_RPM * rated_torque / RPM_range(i) ; 
    end
end


Tractive_force= zeros(length(torque),1);
Speed = zeros(length(torque),1);

Tractive_force(:,1) = torque * FDR / R_wheel;
Speed(:,1) = RPM_range * 2 * pi * R_wheel / (FDR*60); 


%Saving the profile
%save('EMRAX228LC_3.36_0.45ipeak.mat','Gearing_Map','FDR','Tractive_force','Speed','torque','RPM_range')