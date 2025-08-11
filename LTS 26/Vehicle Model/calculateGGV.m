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
        
        % Initialise Decision Variables
        delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);       % steering angle (rad)
        beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);           % body slip (rad)
        Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);               % front slip ratio
        Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);               % rear slip ratio
        dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);           % Yaw rate (rad/s)

        % define initial guess
        prob.set_initial(delta,0);
        prob.set_initial(beta,0);
        prob.set_initial(Sxf,0);
        prob.set_initial(Sxr,0);
        prob.set_initial(dpsi,0);

        % Call Vehicle Model
        outputs = vehicleModel(delta, beta, Sxf, Sxr, dpsi, settings);
    
        % define constraints
        prob.subject_to(PowerOut<=PMaxLimit);
        prob.subject_to(Mz == 0);
        prob.subject_to(ay - V*dpsi == 0);
        prob.solver('ipopt', p_opts, s_opts);
    
        % % acceleration G solver
        prob.minimize(-ax); 
        x = prob.solve(); 
        maxAx = x.value(ax);

        % % braking G solver
        prob.minimize(ax);
        x = prob.solve();
        minAx = x.value(ax);
    
        % % equal spread ax to -ax
        GG.speed(i).ax = [maxAx, linspace(maxAx, minAx, Gnum), minAx];
        GG.speed(i).ay = zeros(1, numel(GG.speed(i).ax));
    
        % Lateral G Solver
        for j = 2:numel(GG.speed(i).ax)-1
            ax_target = GG.speed(i).ax(j);
            prob = casadi.Opti();
            % Decision Variables
            delta = prob.variable(); prob.subject_to(-maxDelta<=delta<=maxDelta);        % steering angle (rad)
            beta = prob.variable(); prob.subject_to(-maxBeta<=beta<=maxBeta);            % body slip (rad)
            Sxf = prob.variable(); prob.subject_to(-maxSxf<=Sxf<=maxSxf);                % front slip ratio
            Sxr = prob.variable(); prob.subject_to(-maxSxr<=Sxr<=maxSxr);                % rear slip ratio
            dpsi = prob.variable(); prob.subject_to(-maxDpsi<=dpsi<=maxDpsi);            % Yaw rate (rad/s)
            % Call Vehicle Model
            vehicle;
            % define objective
            prob.minimize(-ay); % Maximum GG Envelope Radius
    
            % define constraints
            prob.subject_to(Mz == 0);
            prob.subject_to(ax == ax_target);
            prob.subject_to(ay - V*dpsi == 0);
            prob.subject_to(-maxSa<=Saf<=maxSa);
            prob.subject_to(-maxSa<=Sar<=maxSa);
            % optimization results
            prob.solver('ipopt', p_opts, s_opts);
            % security catch in case failure
            try
                x = prob.solve();
                GG.speed(i).ax(j) = x.value(ax);
                GG.speed(i).ay(j) = x.value(ay);
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
    vCar = repmat([GG.speed.speed],Gnum+2,1);
    vCar = vCar(:);
    
    GGV.vCar = vCar';
    GGV.gLong = [GG.speed.ax];
    GGV.gLat = [GG.speed.ay];


end