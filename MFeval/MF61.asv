% Adapted from MFeval

function [Fx, Fy] = MF61(Fz, kappa, alpha, gamma, Vcx, pres, tirParams)
    useLimitsCheck = 1;

    if useLimitsCheck
        % Maximum/Minimum Vertical Force
        FZMIN = tirParams.FZMIN;
        FZMAX = tirParams.FZMAX;
        
        % Maximum/Minimum Slip Ratio
        KPUMIN = tirParams.KPUMIN;
        KPUMAX = tirParams.KPUMAX;

        % Maximum/Minimum Slip Angle
        ALPMIN = tirParams.ALPMIN;
        AlPMAX = tirParams.ALPMIN;

        % Maximum/Minimum Camber Angle
        CAMMIN = tirParams.CAMMIN;
        CAMMAX = tirParams.CAMMAX;

        % Minimum Speed
        VXLOW = tir.param.VXLOW;

        % Maximum/Minimum Inflation Pressure
        PRESMIN = tirParams.PRESMIN;
        PRESMAX = tireParams.PRESMAX;

        % Flag low speed
        isLowSpeed = (abs(Vcx) <= VXLOW);

        % Create a vector with numbers in [0,1] to apply a reduction factor
        % with smooth transition
        Wvlow = 0.5 .* (1 + cos(pi .* (Vcx(isLowSpeed) ./ VXLOW))); % (page 441 - 9.126)
        reductionSmooth = 1 - Wvlow; 

        % Create a vector with numbers in [0,1] to apply a linear reduction
        % toward 0
        reductionLinear = abs(Vcx(isLowSpeed)/VXLOW);
        
        % Create a vector with numbers in [0,1] to apply a reduction factor
        % using a sharp decrease toward 0, but smooth transition toward
        % VXLOW
        reductionSharp = sin(reductionLinear .* (pi/2));

        % Calculate lateral speed for steady state
        Vsy = tan(alpha) .* Vcx;

        % Sum longitudinal and lateral speeds
        speedSum = abs(Vcx) + abs(Vsy);

        % The slip angle also suffers a reduction when Vx + Vy < VXLOW
        isLowSpeedAlpha = (speedSum < VXLOW);

        % Create a vector with numbers in [0,1] to apply a linear reduction
        % toward 0 for alpha
        reductionLinearAlpha = speedSum(isLowSpeedAlpha) / VXLOW;

        % Apply reductionLinear and reductionLinearAlpha to kappa and alpha
        kappa(isLowSpeed) = kappa(isLowSpeed) .* reductionLinear;
        alpha(isLowSpeedAlpha) = alpha(isLowSpeedAlpha) .* reductionLinearAlpha;

        % Check limits
        isLowSlip = (alpha < ALPMIN);
        alpha(isLowSlip) = real(ALPMIN);
        if any(isLowSlip)
            warning ('Solver:Limits:Exceeded',['Slip angle below ',...
                'the limit. Values have been saturated.']);
        end

        isHighSlip = (alpha > ALPMAX);
        alpha(isHighSlip) = real(ALPMAX);
        if any(isHighSlip)
            warning ('Solver:Limits:Exceeded',['Slip angle above ',...
                'the limit. Values have been saturated.']);
        end


    end
