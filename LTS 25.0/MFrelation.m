% This references the 3rd edition of pacejka Tyre and Vehicle Dynamics
% We first derive the basic variables Fz(vertical load), kappa(longitudinal slip), 
% alpha(side slip angle), gamma(inclination angle), Vcx(forward velocity of contact patch), 
% p (tire pressure)
function [] = MFrelation(tirParams, Fz, kappa, alpha, gamma, Vcx, p)

    useLimitsCheck = true; % true sets parameters within the testing conditions, false allows extrapolation
    useAlphaStar = true;
    userDynamics = 0; % solve in steady state

    
    % Create a copy of the variables (u stands for unlimited)
    uFz = Fz;
    ukappa = kappa;
    ukappaLow = kappa;
    ualpha = alpha;
    ugamma = gamma;
    % uphit = phit;
    uVcx = Vcx;
    

    if useLimitsCheck
        % Turn slip modifications
        % phit = phit.*cos(alpha); % Empirically discovered
        
        % Minimum Speed
        VXLOW = tirParams.VXLOW;
        
        % Inflation Pressure Range
        PRESMIN = tirParams.PRESMIN;
        PRESMAX = tirParams.PRESMAX;
        
        % Vertical Force Range
        FZMIN = tirParams.FZMIN;
        FZMAX = tirParams.FZMAX;
        
        % Slip Angle Range
        ALPMIN = tirParams.ALPMIN;
        ALPMAX = tirParams.ALPMAX;
        
        % Inclination Angle Range
        CAMMIN = tirParams.CAMMIN;
        CAMMAX = tirParams.CAMMAX;
        
        % Long Slip Range
        KPUMIN = tirParams.KPUMIN;
        KPUMAX = tirParams.KPUMAX;
        
        % Low Speed Model:
        % Create a reduction factor for low speed and standstill
        
        % Logic to flag if the speed it's below the limit
        isLowSpeed = abs(Vcx) <= VXLOW;
        
        % Create a vector with numbers between 0 and 1 to apply a
        % reduction factor with smooth transitions.
        Wvlow = 0.5.*(1+cos(pi().*(Vcx(isLowSpeed)./VXLOW)));
        reductionSmooth = 1-Wvlow;
        
        % Create a vector with numbers between 0 and 1 to apply a
        % linear reduction toward zero
        reductionLinear = abs(Vcx(isLowSpeed)/VXLOW);
        
        % Create a vector with numbers between 0 and 1 to apply a
        % reduction factor using a sharp decrease toward zero but
        % smooth transition toward VXLOW
        reductionSharp = sin(reductionLinear.*(pi/2));
        
        % ukappaLow is equal to ukappa but with low speed
        % correction. This is only used to export kappa
        ukappaLow(isLowSpeed) = real(ukappaLow(isLowSpeed).*reductionLinear);
        
        % If Vcx is close to zero, use linear reduction
        % phit(isLowSpeed) = phit(isLowSpeed).*reductionLinear;
        
        if userDynamics == 0
            % Calculate the lateral speed if userDynamics is zero
            % (called from MATLAB)
            Vsy = tan(alpha).*Vcx;
        else
            % Dynamics is 1 or 3 (called from Simulink)
            % Grab Vsy from inputs to avoid Vsy = 0 when Vx = 0
            Vsy = inputs(:,9);
        end % Vsy calculation
        
        % If the speed is negative, the turn slip is also negative
        isNegativeSpeed = Vcx<0;
        % phit(isNegativeSpeed) = -phit(isNegativeSpeed);
        
        % Sum the forward and lateral speeds
        speedSum = abs(Vcx) + abs(Vsy);
        
        % The slip angle also suffers a reduction when the sum of
        % Vx and Vy is less than VXLOW
        isLowSpeedAlpha = speedSum < VXLOW;
        
        % Create a vector with numbers between 0 and 1 to apply a
        % linear reduction toward zero for alpha
        reductionLinear_alpha = speedSum(isLowSpeedAlpha)/VXLOW;
        
        if userDynamics == 0
            % Solve only when is mfeval from MATLAB
            kappa(isLowSpeed) = kappa(isLowSpeed).*reductionLinear;
        end % if userDynamics == 0
        
        if userDynamics <= 1
            % Solve for steady state (both MATLAB and Simulink)
            % If Vcx is close to zero then SA is also 0, linear
            % reduction
            alpha(isLowSpeedAlpha) = alpha(isLowSpeedAlpha).*reductionLinear_alpha;
        end % if steady state
        
        % Check Slip Angle limits
        isLowSlip = alpha < ALPMIN;
        alpha(isLowSlip) = real(ALPMIN);
        
        isHighSlip = alpha > ALPMAX;
        alpha(isHighSlip) = real(ALPMAX);
        
        % Check camber limits
        isLowCamber = gamma < CAMMIN;
        gamma(isLowCamber) = real(CAMMIN);
        
        isHighCamber = gamma > CAMMAX;
        gamma(isHighCamber) = real(CAMMAX);
        
        % Check Fz limits
        isHighLoad = Fz > FZMAX;
        Fz(isHighLoad) = real(FZMAX);
        
        % Create a copy of Fz and apply the low limit.
        % This is only used in some Moments equations
        Fz_lowLimit = Fz;
        isLowLoad = Fz < FZMIN;
        Fz_lowLimit(isLowLoad) = real(FZMIN);
        
        % Check pressure limits
        isLowPressure = p < PRESMIN;
        p(isLowPressure) = real(PRESMIN);
        
        isHighPressure = p > PRESMAX;
        p(isHighPressure) = real(PRESMAX);
        
        % Check slip ratio limits
        isLowSlipR = kappa < KPUMIN;
        kappa(isLowSlipR) = real(KPUMIN);
        
        isHighSlipR = kappa > KPUMAX;
        kappa(isHighSlipR) = real(KPUMAX);
        
        % Flag if anything is out of range.
        if any(isLowSlip)
            warning ('Solver:Limits:Exceeded',['Slip angle below ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isHighSlip)
            warning ('Solver:Limits:Exceeded',['Slip angle above ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isLowCamber)
            warning ('Solver:Limits:Exceeded',['Inclination angle below ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isHighCamber)
            warning ('Solver:Limits:Exceeded',['Inclination angle above ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isLowLoad)
            warning ('Solver:Limits:Exceeded',['Vertical load below ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isHighLoad)
            warning ('Solver:Limits:Exceeded',['Vertical load above ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isLowPressure)
            warning ('Solver:Limits:Exceeded',['Pressure below ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isHighPressure)
            warning ('Solver:Limits:Exceeded',['Pressure above ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isLowSlipR)
            warning ('Solver:Limits:Exceeded',['Slip ratio below ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isHighSlipR)
            warning ('Solver:Limits:Exceeded',['Slip ratio above ',...
                'the limit. Values have been saturated.']);
        end
        
        if any(isLowSpeed)
            warning('Solver:Limits:Exceeded',['Speed input VX below ',...
                'the limit. Low speed mode activated.']);
        end
        
    else
        % Not using limits checks
        isLowSpeed = false(length(Fz),1);
        reductionSmooth = ones(length(Fz),1);
        reductionSharp = ones(length(Fz),1);
        reductionLinear = ones(length(Fz),1);
        isLowSpeedAlpha = false(length(Fz),1);
        reductionLinear_alpha = ones(length(Fz),1);
    end % if useLimitsCheck
    

    % IMPORTANT NOTE: Vx = Vcx [Eqn (7.4) Page 331 - Book]
    % It is assumed that the difference between the wheel centre
    % longitudinal velocity Vx and the longitudinal velocity Vcx of
    % the contact centre is negligible

    % Parameters not specified in the TIR file
    % Used to avoid low speed singularity
    epsilon  = 1e-6; % [Eqn (4.E6a) Page 178 - Book]
    epsilonv = epsilon;
    epsilonx = epsilon;
    epsilonk = epsilon;
    epsilony = epsilon;

    % Unpack Parameters
    % tirParams are fitted parameters from TIR file
    V0 = tirParams.LONGVL; %Nominal speed
    pi0	= tirParams.NOMPRES; %Nominal tyre inflation pressure
    Fz0	= tirParams.FNOMIN; %Nominal wheel load
    LFZO = tirParams.LFZO; % Scale factor of nominal (rated) load
    LMUX = tirParams.LMUX; % Scale factor of Fx peak friction coefficient
    LMUY = tirParams.LMUY; % Scale factor of Fy peak friction coefficient

    % New scaling factor in Pacejka 2012 with it's default value.
    % This scaling factor is not present in the standard MF6.1 TIR
    % files.
    LMUV = 0; % Scale factor with slip speed Vs decaying friction

    %{
    % postProInputs, internalParams and modes are from parseInputs
    Fz = postProInputs.Fz;
    kappa = postProInputs.kappa;
    alpha = postProInputs.alpha;
    gamma = postProInputs.gamma;
    Vcx = postProInputs.uVcx;
    p = postProInputs.p;
    %}

    % epsilonv = internalParams.epsilonv;

    % Velocities in point S (slip point)
    % Note: why use absolute of longitudinal velocity of the contact patch
    Vsx = -kappa.*abs(Vcx); % [Eqn (4.E5) Page 181 - Book]
    Vsy = tan(alpha).*abs(Vcx); % [Eqn (2.12) Page 67 - Book] and [(4.E3) Page 177 - Book]
    % Important Note:
    % Due to the ISO sign convention, equation 2.12 does not need a
    % negative sign. The Pacejka book is written in adapted SAE.
    Vs = sqrt(Vsx.^2+Vsy.^2); % [Eqn (3.39) Page 102 - Book] -> Slip velocity of the slip point S

    % Velocities in point C (contact)
    Vcy = Vsy; % Assumption from page 67 of the book, paragraph above Eqn (2.11)
    Vc = sqrt(Vcx.^2+Vcy.^2); % Velocity of the wheel contact centre C, Not described in the book but is the same as [Eqn (3.39) Page 102 - Book]

    % Effect of having a tire with a different nominal load
    Fz0_prime =  LFZO.*Fz0; % [Eqn (4.E1) Page 177 - Book]

    % Normalized change in vertical load
    dfz = (Fz - Fz0_prime)./Fz0_prime; % [Eqn (4.E2a) Page 177 - Book]

    % Normalized change in inflation pressure
    dpi = (p - pi0)./pi0; % [Eqn (4.E2b) Page 177 - Book]

    % Use of star (*) definition. Only valid for the book
    % implementation. TNO MF-Tyre does not use this.
    if useAlphaStar
        alpha_star = tan(alpha).*sign(Vcx); % [Eqn (4.E3) Page 177 - Book]
        gamma_star = sin(gamma); % [Eqn (4.E4) Page 177 - Book]
    else
        alpha_star = alpha;
        gamma_star = gamma;
    end % if useAlphaStar

    % For the aligning torque at high slip angles
    signVc = sign(Vc);
    signVc(signVc==0) = 1;
    Vc_prime = Vc + epsilonv.*signVc; % [Eqn (4.E6a) Page 178 - Book] [sign(Vc) term explained on page 177]

    alpha_prime = acos(Vcx./Vc_prime); % [Eqn (4.E6) Page 177 - Book]

    % Slippery surface with friction decaying with increasing (slip) speed
    LMUX_star = LMUX./(1 + LMUV.*Vs./V0); % [Eqn (4.E7) Page 179 - Book]
    LMUY_star = LMUY./(1 + LMUV.*Vs./V0); % [Eqn (4.E7) Page 179 - Book]

    % Digressive friction factor
    % On Page 179 of the book is suggested Amu = 10, but after
    % comparing the use of the scaling factors against TNO, Amu = 1
    % was giving perfect match
    Amu = 1;
    LMUX_prime = Amu.*LMUX_star./(1+(Amu-1).*LMUX_star); % [Eqn (4.E8) Page 179 - Book]
    LMUY_prime = Amu.*LMUY_star./(1+(Amu-1).*LMUY_star); % [Eqn (4.E8) Page 179 - Book]

    % no need to pack outputs, will be included in the same function
    %{
    starVar.alpha_star = alpha_star;
    starVar.gamma_star = gamma_star;
    starVar.LMUX_star = LMUX_star;
    starVar.LMUY_star = LMUY_star;
    starVar.gamma_star = gamma_star;

    primeVar.Fz0_prime = Fz0_prime;
    primeVar.alpha_prime = alpha_prime;
    primeVar.LMUX_prime = LMUX_prime;
    primeVar.LMUY_prime = LMUY_prime;

    incrVar.dfz = dfz;
    incrVar.dpi = dpi;

    slipVel.Vsx = Vsx;
    slipVel.Vsy = Vsy;
    %}

    % Fx0 pure longitudinal slip

    %[SCALING_COEFFICIENTS]
    LCX 	= tirParams.LCX ; % Scale factor of Fx shape factor
    LEX  	= tirParams.LEX ; % Scale factor of Fx curvature factor
    LKX  	= tirParams.LKX ; % Scale factor of Fx slip stiffness
    LHX  	= tirParams.LHX ; % Scale factor of Fx horizontal shift
    LVX  	= tirParams.LVX ; % Scale factor of Fx vertical shift

    %[LONGITUDINAL_COEFFICIENTS]
    PCX1  	=  tirParams.PCX1 ; %Shape factor Cfx for longitudinal force
    PDX1  	=  tirParams.PDX1 ; %Longitudinal friction Mux at Fznom
    PDX2  	=  tirParams.PDX2 ; %Variation of friction Mux with load
    PDX3  	=  tirParams.PDX3 ; %Variation of friction Mux with camber squared
    PEX1  	=  tirParams.PEX1 ; %Longitudinal curvature Efx at Fznom
    PEX2  	=  tirParams.PEX2 ; %Variation of curvature Efx with load
    PEX3  	=  tirParams.PEX3 ; %Variation of curvature Efx with load squared
    PEX4  	=  tirParams.PEX4 ; %Factor in curvature Efx while driving
    PKX1  	=  tirParams.PKX1 ; %Longitudinal slip stiffness Kfx./Fz at Fznom
    PKX2  	=  tirParams.PKX2 ; %Variation of slip stiffness Kfx./Fz with load
    PKX3  	=  tirParams.PKX3 ; %Exponent in slip stiffness Kfx./Fz with load
    PHX1  	=  tirParams.PHX1 ; %Horizontal shift Shx at Fznom
    PHX2  	=  tirParams.PHX2 ; %Variation of shift Shx with load
    PVX1  	=  tirParams.PVX1 ; %Vertical shift Svx./Fz at Fznom
    PVX2  	=  tirParams.PVX2 ; %Variation of shift Svx./Fz with load
    PPX1  	=  tirParams.PPX1 ; %linear influence of inflation pressure on longitudinal slip stiffness
    PPX2  	=  tirParams.PPX2 ; %quadratic influence of inflation pressure on longitudinal slip stiffness
    PPX3  	=  tirParams.PPX3 ; %linear influence of inflation pressure on peak longitudinal friction
    PPX4  	=  tirParams.PPX4 ; %quadratic influence of inflation pressure on peak longitudinal friction

    % not using turn slip
    zeta1 = 1;
    Cx = PCX1.*LCX; % (> 0) (4.E11)
    mux = (PDX1 + PDX2.*dfz).*(1 + PPX3.*dpi + PPX4.*dpi.^2).*(1 - PDX3.*gamma.^2).*LMUX_star; % (4.E13)
    mux(Fz==0) = 0; % Zero Fz correction
    Dx = mux.*Fz.*zeta1; % (> 0) (4.E12)
    Kxk = Fz.*(PKX1 + PKX2.*dfz).*exp(PKX3.*dfz).*(1 + PPX1.*dpi + PPX2.*dpi.^2).*LKX;  % (= BxCxDx = dFxo./dkx at kappax = 0) (= Cfk) (4.E15)
    
    signDx = sign(Dx);
    signDx(signDx == 0) = 1; % If [Dx = 0] then [sign(0) = 0]. This is done to avoid [Kxk / 0 = NaN] in Eqn 4.E16
    
    Bx = Kxk./(Cx.*Dx + epsilonx.*signDx); % (4.E16) [sign(Dx) term explained on page 177]
    SHx = (PHX1 + PHX2.*dfz).*LHX; % (4.E17)
    SVx = Fz.*(PVX1 + PVX2.*dfz).*LVX.*LMUX_prime.*zeta1; % (4.E18)
    
    % Low speed model
    if any(isLowSpeed)
        SVx(isLowSpeed) = SVx(isLowSpeed).*internalParams.reductionSmooth;
        SHx(isLowSpeed) = SHx(isLowSpeed).*internalParams.reductionSmooth;
    end % if isLowSpeed
    
    kappax = kappa + SHx; % (4.E10)
    Ex = (PEX1 + PEX2.*dfz + PEX3.*dfz.^2).*(1 - PEX4.*sign(kappax)).*LEX; % (<=1) (4.E14)
    
    % Limits check
    if(useLimitsCheck)
        if(any(Ex > 1))
            warning('Solver:CoeffChecks:Ex','Ex over limit (>1), Eqn(4.E14)')
            Ex(Ex > 1) = 1;
        end % if Ex > 1
    end % if useLimitsCheck
    
    % Pure longitudinal force
    Fx0 = Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+SVx; % (4.E9)
    
    % Fy0 pure lateral slip condition

    %[SCALING_COEFFICIENTS]
    LCY   = tirParams.LCY   ; % Scale factor of Fy shape factor
    LEY   = tirParams.LEY   ; % Scale factor of Fy curvature factor
    LKY   = tirParams.LKY   ; % Scale factor of Fy cornering stiffness
    LHY   = tirParams.LHY   ; % Scale factor of Fy horizontal shift
    LVY   = tirParams.LVY   ; % Scale factor of Fy vertical shift
    LKYC  = tirParams.LKYC  ; % Scale factor of camber force stiffness
    
    %[LATERAL_COEFFICIENTS]
    PCY1  =  tirParams.PCY1 	; %Shape factor Cfy for lateral forces
    PDY1  =  tirParams.PDY1 	; %Lateral friction Muy
    PDY2  =  tirParams.PDY2 	; %Variation of friction Muy with load
    PDY3  =  tirParams.PDY3 	; %Variation of friction Muy with squared camber
    PEY1  =  tirParams.PEY1 	; %Lateral curvature Efy at Fznom
    PEY2  =  tirParams.PEY2 	; %Variation of curvature Efy with load
    PEY3  =  tirParams.PEY3 	; %Zero order camber dependency of curvature Efy
    PEY4  =  tirParams.PEY4 	; %Variation of curvature Efy with camber
    PEY5  =  tirParams.PEY5 	; %Variation of curvature Efy with camber squared
    PKY1  =  tirParams.PKY1 	; %Maximum value of stiffness Kfy./Fznom
    PKY2  =  tirParams.PKY2 	; %Load at which Kfy reaches maximum value
    PKY3  =  tirParams.PKY3 	; %Variation of Kfy./Fznom with camber
    PKY4  =  tirParams.PKY4 	; %Curvature of stiffness Kfy
    PKY5  =  tirParams.PKY5 	; %Peak stiffness variation with camber squared
    PKY6  =  tirParams.PKY6 	; %Fy camber stiffness factor
    PKY7  =  tirParams.PKY7 	; %Vertical load dependency of camber stiffness
    PHY1  =  tirParams.PHY1 	; %Horizontal shift Shy at Fznom
    PHY2  =  tirParams.PHY2 	; %Variation of shift Shy with load
    PVY1  =  tirParams.PVY1 	; %Vertical shift in Svy./Fz at Fznom
    PVY2  =  tirParams.PVY2 	; %Variation of shift Svy./Fz with load
    PVY3  =  tirParams.PVY3 	; %Variation of shift Svy./Fz with camber
    PVY4  =  tirParams.PVY4 	; %Variation of shift Svy./Fz with camber and load
    PPY1  =  tirParams.PPY1 	; %influence of inflation pressure on cornering stiffness
    PPY2  =  tirParams.PPY2 	; %influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
    PPY3  =  tirParams.PPY3 	; %linear influence of inflation pressure on lateral peak friction
    PPY4  =  tirParams.PPY4 	; %quadratic influence of inflation pressure on lateral peak friction
    PPY5  =  tirParams.PPY5 	; %Influence of inflation pressure on camber stiffness
    
    zeta2 = 1;
    zeta3 = 1;
    zeta0 = 1;
    zeta4 = 1;

    Kya = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(gamma_star)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*gamma_star.^2).*(1+PPY2.*dpi)))).*zeta3.*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
    SVyg = Fz.*(PVY3 + PVY4.*dfz).*gamma_star.* LKYC .* LMUY_prime .* zeta2; % (4.E28)
    Kyg0 = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi).*LKYC; % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)

    signKya = sign(Kya);
    signKya(signKya == 0) = 1; % If [Kya = 0] then [sign(0) = 0]. This is done to avoid [num / 0 = NaN] in Eqn 4.E27

    SHy = (PHY1 + PHY2.*dfz).* LHY + ((Kyg0 .*gamma_star - SVyg)./(Kya + epsilonk.*signKya)).*zeta0 +zeta4 -1; % (4.E27) [sign(Kya) term explained on page 177]
    SVy = Fz.*(PVY1 + PVY2.*dfz).*LVY.*LMUY_prime.*zeta2 + SVyg; % (4.E29)
    
    % Low speed model
    isLowSpeed = modes.isLowSpeed;
    if any(isLowSpeed)
        SVy(isLowSpeed) = SVy(isLowSpeed).*internalParams.reductionSmooth;
        SHy(isLowSpeed) = SHy(isLowSpeed).*internalParams.reductionSmooth;
    end % if isLowSpeed
    
    alphay = alpha_star + SHy; % (4.E20)
    Cy = PCY1.*LCY; % (> 0) (4.E21)
    muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 - PDY3.*gamma_star.^2).*LMUY_star; % (4.E23)
    Dy = muy.*Fz.*zeta2; % (4.E22)
    signAlphaY = sign(alphay);
    signAlphaY(signAlphaY == 0) = 1;
    Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*gamma_star.^2 - (PEY3 + PEY4.*gamma_star).*signAlphaY).*LEY; % (<=1)(4.E24)
    
    % Limits check
    if(useLimitsCheck)
        if(any(Ey > 1))
            warning('Solver:CoeffChecks:Ey','Ey over limit (>1), Eqn(4.E24)')
            Ey(Ey > 1) = 1;
        end % if Ey > 1
    end % if useLimitsCheck
    
    signDy = sign(Dy);
    signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya / 0 = NaN] in Eqn 4.E26
    
    By = Kya./(Cy.*Dy + epsilony.*signDy); % (4.E26) [sign(Dy) term explained on page 177]
    
    Fy0 = Dy .* sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay - atan(By.*alphay))))+ SVy; % (4.E19)
    
    % Backward speed check for alpha_star
    if(useAlphaStar)
        Fy0(Vcx < 0) = -Fy0(Vcx < 0);
    end % if useAlphaStar
    