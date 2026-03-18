% Adapted from MFeval

function [Fx, Fy] = SM_MF61(Fz, kappa, alpha, gamma, pres)
    %% Unpack Parameters for processing
    tirParams = mfeval.readTIR("D:\FSAE26\LTS 26\Vehicle Model\43105_18x7.5_10_R25B_7.tir");
    % tirParams are fitted parameters from TIR file
    % V0 = tirParams.LONGVL; %Nominal speed
    pi0	= tirParams.NOMPRES; %Nominal tyre inflation pressure
    Fz0	= tirParams.FNOMIN; %Nominal wheel load
    LFZO = tirParams.LFZO; % Scale factor of nominal (rated) load
    LMUX = tirParams.LMUX; % Scale factor of Fx peak friction coefficient
    LMUY = tirParams.LMUY; % Scale factor of Fy peak friction coefficient

    % epsilonv = internalParams.epsilonv;

    alpha_star = tan(alpha); % [Eqn (4.E3) Page 177 - Book]
    gamma_star = sin(gamma); % [Eqn (4.E4) Page 177 - Book]

    % Digressive friction factor
    % On Page 179 of the book is suggested Amu = 10, but after
    % comparing the use of the scaling factors against TNO, Amu = 1
    % was giving perfect match
    Amu = 1;
    LMUX_prime = Amu.*LMUX./(1+(Amu-1).*LMUX); % [Eqn (4.E8) Page 179 - Book]
    LMUY_prime = Amu.*LMUY./(1+(Amu-1).*LMUY); % [Eqn (4.E8) Page 179 - Book]

    % Effect of having a tire with a different nominal load
    Fz0_prime =  LFZO.*Fz0; % [Eqn (4.E1) Page 177 - Book]
    % Normalized change in vertical load
    dfz = (Fz - Fz0_prime)./Fz0_prime; % [Eqn (4.E2a) Page 177 - Book]
    % Normalized change in inflation pressure
    dpi = (pres - pi0)./pi0; % [Eqn (4.E2b) Page 177 - Book]
    epsilon  = 1e-6;

    sigma = sqrt( kappa.^2 + alpha_star.^2 + epsilon.^2);

    %% Pure Longitudinal Slip Fx0 and Combined Longitudinal Slip Fx
    %[SCALING_COEFFICIENTS]
    LCX 	= tirParams.LCX ; % Scale factor of Fx shape factor
    LEX  	= tirParams.LEX ; % Scale factor of Fx curvature factor
    LKX  	= tirParams.LKX ; % Scale factor of Fx slip stiffness
    LHX  	= tirParams.LHX ; % Scale factor of Fx horizontal shift
    LVX  	= tirParams.LVX ; % Scale factor of Fx vertical shift
    LXAL    = tirParams.LXAL ; % Scale factor of alpha influence on Fx

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

    zeta1 = 1;
    Cx = PCX1.*LCX; % (> 0) (4.E11)
    mux = (PDX1 + PDX2.*dfz).*(1 + PPX3.*dpi + PPX4.*dpi.^2).*(1 - PDX3.*gamma.^2).*LMUX; % (4.E13)
    % mux(Fz==0) = 0; % Zero Fz correction
    Dx = mux.*Fz.*zeta1; % (> 0) (4.E12)
    Kxk = Fz.*(PKX1 + PKX2.*dfz).*exp(PKX3.*dfz).*(1 + PPX1.*dpi + PPX2.*dpi.^2).*LKX;  % (= BxCxDx = dFxo./dkx at kappax = 0) (= Cfk) (4.E15)
    signDx = sign(Dx);
    % signDx(signDx == 0) = 1; % If [Dx = 0] then [sign(0) = 0]. This is done to avoid [Kxk / 0 = NaN] in Eqn 4.E16
    Bx = Kxk./(Cx.*Dx + epsilon.*signDx); % (4.E16) [sign(Dx) term explained on page 177]
    SHx = (PHX1 + PHX2.*dfz).*LHX; % (4.E17)
    SVx = Fz.*(PVX1 + PVX2.*dfz).*LVX.*LMUX_prime.*zeta1; % (4.E18)
    kappax = kappa + SHx; % (4.E10)
    Ex = (PEX1 + PEX2.*dfz + PEX3.*dfz.^2).*(1 - PEX4.*sign(kappax)).*LEX; % (<=1) (4.E14)
    % if(any(Ex > 1))
    %     warning('Solver:CoeffChecks:Ex','Ex over limit (>1), Eqn(4.E14)')
    %     Ex(Ex > 1) = 1;
    % end % if Ex > 1
    Fx0 = Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+SVx; % (4.E9)
    Fx = (abs(kappa)./sigma).*Fx0;
   
    %% Pure Lateral Slip Fy0 and Combined Lateral Slip Fy
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

    Cy = PCY1.*LCY; % (> 0) (4.E21)
    muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 - PDY3.*gamma_star.^2).*LMUY; % (4.E23)
    Dy = muy.*Fz.*zeta2; % (4.E22)
    Kya = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(gamma_star)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*gamma_star.^2).*(1+PPY2.*dpi)))).*zeta3.*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
    signDy = sign(Dy);
    % signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya / 0 = NaN] in Eqn 4.E26
    By = Kya./(Cy.*Dy + epsilon.*signDy); % (4.E26) [sign(Dy) term explained on page 177]
    SVyg = Fz.*(PVY3 + PVY4.*dfz).*gamma_star.* LKYC .* LMUY_prime .* zeta2; % (4.E28)
    Kyg0 = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi).*LKYC; % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
    signKya = sign(Kya);
    % signKya(signKya == 0) = 1; % If [Kya = 0] then [sign(0) = 0]. This is done to avoid [num / 0 = NaN] in Eqn 4.E27
    SHy = (PHY1 + PHY2.*dfz).* LHY + ((Kyg0 .*gamma_star - SVyg)./(Kya + epsilon.*signKya)).*zeta0 +zeta4 -1; % (4.E27) [sign(Kya) term explained on page 177]
    SVy = Fz.*(PVY1 + PVY2.*dfz).*LVY.*LMUY_prime.*zeta2 + SVyg; % (4.E29)
    alphay = alpha_star + SHy; % (4.E20)
    signAlphaY = sign(alphay);
    % signAlphaY(signAlphaY == 0) = 1;
    Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*gamma_star.^2 - (PEY3 + PEY4.*gamma_star).*signAlphaY).*LEY; % (<=1)(4.E24)
    % if(any(Ey > 1))
    %     warning('Solver:CoeffChecks:Ey','Ey over limit (>1), Eqn(4.E24)')
    %     Ey(Ey > 1) = 1;
    % end % if Ey > 1
    Fy0 = Dy.*sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay-atan(By.*alphay))))+ SVy; % (4.E19)
    Fy = (abs(alpha_star)./sigma) .* Fy0;
    
end