function GGV = calculateGGV(settings, GGV_settings)

import casadi.*

for i = 1:numel(GGV_settings.velocityRange)

    velocity = GGV_settings.velocityRange(i);

    % Angle Range
    alphaRange = linspace(-pi/2, pi/2, GGV_settings.Gnum);
    
    tStart = tic;
    
    for j = 1:numel(alphaRange)
        alpha = alphaRange(j);
        
        % Create Optimisation Problem
        opti = casadi.Opti();
        
        % decision variables & box constraints
        inputs = struct;
        
        inputs.p     = opti.variable();            opti.subject_to(0<=inputs.p<=10);                       % Radius of GG envelope (g)
        inputs.delta = opti.variable();            opti.subject_to(-settings.bounds.maxDelta<=inputs.delta<=settings.bounds.maxDelta);             % steering angle (rad)
        inputs.beta = opti.variable();             opti.subject_to(-settings.bounds.maxBeta<=inputs.beta<=settings.bounds.maxBeta);              % sideslip angle (rad)
        inputs.kappa_fl = opti.variable();         opti.subject_to(settings.bounds.minSxf<=inputs.kappa_fl<=settings.bounds.maxSxf)           % FL wheel angular velocity (rad/s)
        inputs.kappa_fr = opti.variable();         opti.subject_to(settings.bounds.minSxf<=inputs.kappa_fr<=settings.bounds.maxSxf)           % FR wheel angular velocity (rad/s)
        inputs.kappa_rl = opti.variable();         opti.subject_to(settings.bounds.minSxr<=inputs.kappa_rl<=settings.bounds.maxSxr)           % RL wheel angular velocity (rad/s)
        inputs.kappa_rr = opti.variable();         opti.subject_to(settings.bounds.minSxr<=inputs.kappa_rr<=settings.bounds.maxSxr)           % RR wheel angular velocity (rad/s)
        
        % Additional Inputs
        inputs.Vx = velocity;         % Forward Velocity (m/s)
        inputs.ay_control = inputs.p * 9.81 * cos(alpha);        % Lateral Acceleration - imposed as a result of GG envelope radius and angle alpha
        inputs.ax_control = inputs.p * 9.81 * sin(alpha);        % Longitudinal Acceleration - imposed as a result of GG envelope radius and angle alpha
        inputs.curvature = inputs.ay_control / inputs.Vx^2;
        inputs.yawRate = inputs.Vx * inputs.curvature;
    
        outputs = FourWheel.vehicleModel(inputs, settings);    
        
        % Common Constraints
        opti.subject_to(outputs.residuals.ay_res==0);
        opti.subject_to(outputs.residuals.ax_res==0);
        opti.subject_to(outputs.residuals.power<=0);
        opti.subject_to(outputs.Mz==0);
        
        % Common Solver Settings
        plugin_opts = struct('print_time',0);
        solver_opts = struct('print_level',0);
        opti.solver('ipopt', plugin_opts, solver_opts);  
        
        % Objective - Maximum GG Radius for this angle alpha
        opti.minimize(-(inputs.p^2));

        try
            sol = opti.solve(); 
        
            results.velocity(i,j) = velocity;
            results.ax(i,j) = sol.value(outputs.ax);
            results.ay(i,j) = sol.value(outputs.ay);

        catch
            results.velocity(i,j) = velocity;
            results.ax(i,j) = NaN;
            results.ay(i,j) = NaN;
            fprintf('Failed at Velocity %0.2f [m/s] and Alpha %0.2f [deg] \n', velocity, rad2deg(alpha))
        end
    
    end
    
    toc(tStart);

end

% For speeds lower than v_min, it can be assumed that the performance
% envelope is unchanged (due to reduced/minimal influence of aero
% performance), of course - this does not extend to the kinematic
% cornering cases i.e. very low speeds, cornering with minimal/no body
% slip angle
% So we take the easy way out of prefixing the acceleration envelope to a
% very low speed, here 5 m/s

GGV.vCar = reshape([repmat(5,1,GGV_settings.Gnum); results.velocity], [], 1);
GGV.gLong = reshape([results.ax(1,:); results.ax], [], 1);
GGV.gLat = reshape([results.ay(1,:); results.ay], [], 1);


figure(1);clf
scatter3(GGV.gLat, GGV.gLong, GGV.vCar,[],GGV.vCar)
xlabel('gLat [m/s^2]')
ylabel('gLong [m/s^2]')
zlabel('vCar [m/s]')

end
