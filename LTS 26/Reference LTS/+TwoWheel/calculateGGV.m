function GGV = calculateGGV(settings, GGV_settings)

    % Function Calculate GGV Performance Envelope for Given Car Parameters
    
    import casadi.*
    
    tic
    % % Create empty performance envelope GG
    GG = struct();
    GG.speed = struct();
    figure
    
    
    % % Steady State Speed Setting
    for i = 1:numel(GGV_settings.velocityRange)
        V = GGV_settings.velocityRange(i);
        GG.speed(i).speed = V;
        
        % Maximum Forward Acceleration
        prob = casadi.Opti();    
        
        % Decision Variables
        delta = prob.variable(); prob.subject_to(-settings.bounds.maxDelta<=delta<=settings.bounds.maxDelta);        % steering angle (rad)
        beta = prob.variable(); prob.subject_to(-settings.bounds.maxBeta<=beta<=settings.bounds.maxBeta);            % body slip (rad)
        Sxf = prob.variable(); prob.subject_to(-settings.bounds.maxSxf<=Sxf<=settings.bounds.maxSxf);                % front slip ratio
        Sxr = prob.variable(); prob.subject_to(-settings.bounds.maxSxr<=Sxr<=settings.bounds.maxSxr);                % rear slip ratio
        dpsi = prob.variable(); prob.subject_to(-settings.bounds.maxDpsi<=dpsi<=settings.bounds.maxDpsi);            % Yaw rate (rad/s)
            
        % define initial guess
        prob.set_initial(delta,0);
        prob.set_initial(beta,0);
        prob.set_initial(Sxf,0);
        prob.set_initial(Sxr,0);
        prob.set_initial(dpsi,0);

        % Call Vehicle Model
        outputs = TwoWheel.vehicleModel(V, delta, beta, Sxf, Sxr, dpsi, settings);
    
        % define constraints
        prob.subject_to(outputs.PowerOut<=settings.powertrain.powerLimit);
        prob.subject_to(outputs.Mz == 0);
        prob.subject_to(outputs.ay - V*dpsi == 0);
        prob.solver('ipopt', settings.IPOPT.p_opts, settings.IPOPT.s_opts);
    
        % % acceleration G solver
        prob.minimize(-outputs.ax); 
        x = prob.solve(); 
        maxAx = x.value(outputs.ax);

        % % braking G solver
        prob.minimize(outputs.ax);
        x = prob.solve();
        minAx = x.value(outputs.ax);
    
        % % equal spread ax to -ax
        GG.speed(i).ax = [maxAx, linspace(maxAx, minAx, GGV_settings.Gnum), minAx];
        GG.speed(i).ay = zeros(1, numel(GG.speed(i).ax));
    
        % Lateral G Solver
        for j = 2:numel(GG.speed(i).ax)-1
            ax_target = GG.speed(i).ax(j);
            prob = casadi.Opti();
            
            % Decision Variables
            delta = prob.variable(); prob.subject_to(-settings.bounds.maxDelta<=delta<=settings.bounds.maxDelta);        % steering angle (rad)
            beta = prob.variable(); prob.subject_to(-settings.bounds.maxBeta<=beta<=settings.bounds.maxBeta);            % body slip (rad)
            Sxf = prob.variable(); prob.subject_to(-settings.bounds.maxSxf<=Sxf<=settings.bounds.maxSxf);                % front slip ratio
            Sxr = prob.variable(); prob.subject_to(-settings.bounds.maxSxr<=Sxr<=settings.bounds.maxSxr);                % rear slip ratio
            dpsi = prob.variable(); prob.subject_to(-settings.bounds.maxDpsi<=dpsi<=settings.bounds.maxDpsi);            % Yaw rate (rad/s)
            
            % Call Vehicle Model
            outputs = TwoWheel.vehicleModel(V, delta, beta, Sxf, Sxr, dpsi, settings);
    
            % define objective
            prob.minimize(-outputs.ay); % Maximum GG Envelope Radius
    
            % define constraints
            prob.subject_to(outputs.Mz == 0);
            prob.subject_to(outputs.ax == ax_target);
            prob.subject_to(outputs.ay - V*dpsi == 0);
            prob.subject_to(-settings.bounds.maxSa<=outputs.Saf<=settings.bounds.maxSa);
            prob.subject_to(-settings.bounds.maxSa<=outputs.Sar<=settings.bounds.maxSa);
            % optimization results
            prob.solver('ipopt', settings.IPOPT.p_opts, settings.IPOPT.s_opts);
            % security catch in case failure
            try
                x = prob.solve();
                GG.speed(i).ax(j) = x.value(outputs.ax);
                GG.speed(i).ay(j) = x.value(outputs.ay);
            catch
                GG.speed(i).ax(j) = NaN;
                GG.speed(i).ay(j) = NaN;
                fprintf("Combined Slip Failed at V - %0.2f [m/s] & Ax - %0.2f [m/s^2] \n", V, ax_target)
            end
        end
        % store maximum ay at each speed
        GG.speed(i).aymax = max(GG.speed(i).ay);
        % 3D plot GG diagram
        z = GG.speed(i).speed* ones(size(GG.speed(i).ax));  
        plot3(GG.speed(i).ay, GG.speed(i).ax, z, 'LineWidth', 1.5)
        hold on
    end
    
    %% Smaller/Cleaner GGV Array
    
    GGV = struct;
    
    % Get Car Forward Velocity into an array that is inline with collapsed
    % acceleration array
    vCar = repmat([GG.speed.speed],GGV_settings.Gnum+2,1);
    vCar = vCar(:);
    
    GGV.vCar = vCar';
    GGV.gLong = [GG.speed.ax];
    GGV.gLat = [GG.speed.ay];

    % For speeds lower than v_min, it can be assumed that the performance
    % envelope is unchanged (due to reduced/minimal influence of aero
    % performance), of course - this does not extend to the kinematic
    % cornering cases i.e. very low speeds, cornering with minimal/no body
    % slip angle

    % so we prefix the calculated GGV with the lowest speeds, change to
    % column vector
    GGV.vCar = [repmat(min(GGV.vCar)-5,1,GGV_settings.Gnum+2), GGV.vCar]';
    GGV.gLong = [GGV.gLong(:,1:GGV_settings.Gnum+2), GGV.gLong]';
    GGV.gLat = [GGV.gLat(:,1:GGV_settings.Gnum+2), GGV.gLat]';

end