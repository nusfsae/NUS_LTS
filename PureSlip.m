%% SETUP
cd("D:\CDE3301");

% [load tyre model parameters]
tirParams = mfeval.readTIR("D:\CDE3301\43105_18x7.5_10_R25B_7.tir");

% [VEHICLE / TYRE PARAMETERS]
radius   = 0.2032;     % m
g        = 9.81;       % m/s^2
mass     = 66;         % kg per tyre
Fz       = mass*g;     % N
error    = 0.0012;     % Taylor residual increment
gamma    = 0;          % rad camber
Pa       = 62052.8;    % Pa inflation pressure

% [process magic formula scaling parameters]
proParams = calculateParams(Fz, Pa, tirParams);

% [FIND OPTIMAL SLIP RATIO (max Fx0)]
a       = 0.05;
d       = 0.45;
tol     = 1e-7;
maxIter = 5000;
[k_star, Fx0_star, iterGold] = goldenSearchMethod(a, d, tol, maxIter, Fz, gamma, tirParams, proParams);

% [MOTOR PARAMETERS]
torqueLim   = 230;     % Nm
FDR         = 3;       % -
momInertia  = 0.0421;  % kg m^2

% [speed limit where torque saturates]
[speedLim, rpmLim] = calculatePeakAccelDomain(k_star, mass, radius, torqueLim, FDR, momInertia, error);

% [SOLVE ODE FOR VEHICLE SPEED PROFILE]
tMax = 10; % s
sol  = findV(tMax, Fx0_star, speedLim, torqueLim, mass, Fz, gamma, radius, FDR, momInertia, tirParams, proParams, error);

% [NEWTON RAPHSON'S TO GET TIME TO 75m]
init        = 0;   % initial guess t=0
targetDist  = 75;  % m
compPoints  = 20;  % segments for Simpson 3/8

[t_star, velEnd, d, iterNewton] = newtonRaph(init, sol, maxIter, tol, targetDist, tMax, compPoints);

testFx0(Fx0_star, k_star, tirParams);
%% [Functions]

function proParams = calculateParams(Fz, Pa, tirParams)
    % tirParams are fitted parameters from TIR file
    V0 = tirParams.LONGVL; %Nominal speed
    pi0	= tirParams.NOMPRES; %Nominal tyre inflation pressure
    Fz0	= tirParams.FNOMIN; %Nominal wheel load
    LFZO = tirParams.LFZO; % Scale factor of nominal (rated) load
    LMUX = tirParams.LMUX; % Scale factor of Fx peak friction coefficient
    LMUY = tirParams.LMUY; % Scale factor of Fy peak friction coefficient

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
    dpi = (Pa - pi0)./pi0; % [Eqn (4.E2b) Page 177 - Book]

    epsilon  = 1e-6;
    
    % Pack processed parameters
    proParams.LMUX_prime = LMUX_prime;
    proParams.LMUY_prime = LMUY_prime;
    proParams.Fz0_prime = Fz0_prime;
    proParams.dfz = dfz;
    proParams.dpi = dpi;
    proParams.epsilon = epsilon;
end

function fxOut = Fx0(Fz, kappa, gamma, tirParams, proParams)
    %[SCALING_COEFFICIENTS]
    LCX     = tirParams.LCX ; % Scale factor of Fx shape factor
    LEX  	= tirParams.LEX ; % Scale factor of Fx curvature factor
    LKX  	= tirParams.LKX ; % Scale factor of Fx slip stiffness
    LHX  	= tirParams.LHX ; % Scale factor of Fx horizontal shift
    LVX  	= tirParams.LVX ; % Scale factor of Fx vertical shift
    LMUX    = tirParams.LMUX ; % Scale factor of Fx peak friction coefficient
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
    
    %[PROCESSED PARAMETERS]
    dfz = proParams.dfz;
    dpi = proParams.dpi;
    LMUX_prime = proParams.LMUX_prime;
    epsilon = proParams.epsilon;

    zeta1 = 1;

    Cx = PCX1.*LCX; % (> 0) (4.E11)
    mux = (PDX1 + PDX2.*dfz).*(1 + PPX3.*dpi + PPX4.*dpi.^2).*(1 - PDX3.*gamma.^2).*LMUX; % (4.E13)
    mux(Fz==0) = 0; % Zero Fz correction
    Dx = mux.*Fz.*zeta1; % (> 0) (4.E12)
    Kxk = Fz.*(PKX1 + PKX2.*dfz).*exp(PKX3.*dfz).*(1 + PPX1.*dpi + PPX2.*dpi.^2).*LKX;  % (= BxCxDx = dFxo./dkx at kappax = 0) (= Cfk) (4.E15)
    signDx = sign(Dx);
    signDx(signDx == 0) = 1; % If [Dx = 0] then [sign(0) = 0]. This is done to avoid [Kxk / 0 = NaN] in Eqn 4.E16
    Bx = Kxk./(Cx.*Dx + epsilon.*signDx); % (4.E16) [sign(Dx) term explained on page 177]
    SHx = (PHX1 + PHX2.*dfz).*LHX; % (4.E17)
    SVx = Fz.*(PVX1 + PVX2.*dfz).*LVX.*LMUX_prime.*zeta1; % (4.E18)
    kappax = kappa + SHx; % (4.E10)
    Ex = (PEX1 + PEX2.*dfz + PEX3.*dfz.^2).*(1 - PEX4.*sign(kappax)).*LEX; % (<=1) (4.E14)
    if(any(Ex > 1))
        warning('Solver:CoeffChecks:Ex','Ex over limit (>1), Eqn(4.E14)')
        Ex(Ex > 1) = 1;
    end % if Ex > 1
    Fx0 = Dx.*sin(Cx.*atan(Bx.*kappax-Ex.*(Bx.*kappax-atan(Bx.*kappax))))+SVx; % (4.E9)
    
    % Pack pure longitudinal parameters
    fxOut.Fx0 = Fx0; % (4.E9)
    fxOut.kappax = kappax; % (4.E10)
    fxOut.Cx = Cx; % (4.E11)
    fxOut.Dx = Dx; % (4.E12)
    fxOut.mux = mux; % (4.E13)
    fxOut.Ex = Ex; % (4.E14)
    fxOut.Kxk = Kxk; % (4.E15) 
    fxOut.Bx = Bx; % (4.E16)
    fxOut.SHx = SHx; % (4.E17)
    fxOut.SVx = SVx; % (4.E18)
end

function fyOut = Fy0(Fz, alpha, sine_gamma, tirParams, proParams)
    %[SCALING_COEFFICIENTS]
    LCY     = tirParams.LCY   ; % Scale factor of Fy shape factor
    LEY     = tirParams.LEY   ; % Scale factor of Fy curvature factor
    LKY     = tirParams.LKY   ; % Scale factor of Fy cornering stiffness
    LHY     = tirParams.LHY   ; % Scale factor of Fy horizontal shift
    LVY     = tirParams.LVY   ; % Scale factor of Fy vertical shift
    LMUY    = tirParams.LMUY  ; % Scale factor of Fy peak friction coefficient
    LKYC    = tirParams.LKYC  ; % Scale factor of camber force stiffness
    
    %[LATERAL_COEFFICIENTS]
    PCY1    =  tirParams.PCY1 	; %Shape factor Cfy for lateral forces
    PDY1    =  tirParams.PDY1 	; %Lateral friction Muy
    PDY2    =  tirParams.PDY2 	; %Variation of friction Muy with load
    PDY3    =  tirParams.PDY3 	; %Variation of friction Muy with squared camber
    PEY1    =  tirParams.PEY1 	; %Lateral curvature Efy at Fznom
    PEY2    =  tirParams.PEY2 	; %Variation of curvature Efy with load
    PEY3    =  tirParams.PEY3 	; %Zero order camber dependency of curvature Efy
    PEY4    =  tirParams.PEY4 	; %Variation of curvature Efy with camber
    PEY5    =  tirParams.PEY5 	; %Variation of curvature Efy with camber squared
    PKY1    =  tirParams.PKY1 	; %Maximum value of stiffness Kfy./Fznom
    PKY2    =  tirParams.PKY2 	; %Load at which Kfy reaches maximum value
    PKY3    =  tirParams.PKY3 	; %Variation of Kfy./Fznom with camber
    PKY4    =  tirParams.PKY4 	; %Curvature of stiffness Kfy
    PKY5    =  tirParams.PKY5 	; %Peak stiffness variation with camber squared
    PKY6    =  tirParams.PKY6 	; %Fy camber stiffness factor
    PKY7    =  tirParams.PKY7 	; %Vertical load dependency of camber stiffness
    PHY1    =  tirParams.PHY1 	; %Horizontal shift Shy at Fznom
    PHY2    =  tirParams.PHY2 	; %Variation of shift Shy with load
    PVY1    =  tirParams.PVY1 	; %Vertical shift in Svy./Fz at Fznom
    PVY2    =  tirParams.PVY2 	; %Variation of shift Svy./Fz with load
    PVY3    =  tirParams.PVY3 	; %Variation of shift Svy./Fz with camber
    PVY4    =  tirParams.PVY4 	; %Variation of shift Svy./Fz with camber and load
    PPY1    =  tirParams.PPY1 	; %influence of inflation pressure on cornering stiffness
    PPY2    =  tirParams.PPY2 	; %influence of inflation pressure on dependency of nominal tyre load on cornering stiffness
    PPY3    =  tirParams.PPY3 	; %linear influence of inflation pressure on lateral peak friction
    PPY4    =  tirParams.PPY4 	; %quadratic influence of inflation pressure on lateral peak friction
    PPY5    =  tirParams.PPY5 	; %Influence of inflation pressure on camber stiffness
    
    zeta0 = 1;
    zeta2 = 1;
    zeta3 = 1;
    zeta4 = 1;

    %[PROCESSED PARAMETERS]
    Fz0_prime = proParams.Fz0_prime;
    dfz = proParams.dfz;
    dpi = proParams.dpi;
    LMUY_prime = proParams.LMUY_prime;
    epsilon = proParams.epsilon;

    Cy = PCY1.*LCY; % (> 0) (4.E21)
    muy = (PDY1 + PDY2 .* dfz).*(1 + PPY3.*dpi + PPY4 .*dpi.^2).*(1 - PDY3.*sine_gamma.^2).*LMUY; % (4.E23)
    Dy = muy.*Fz.*zeta2; % (4.E22)
    Kya = PKY1.*Fz0_prime.*(1 + PPY1.*dpi).*(1 - PKY3.*abs(sine_gamma)).*sin(PKY4.*atan((Fz./Fz0_prime)./((PKY2+PKY5.*sine_gamma.^2).*(1+PPY2.*dpi)))).*zeta3.*LKY; % (= ByCyDy = dFyo./dalphay at alphay = 0) (if gamma =0: =Kya0 = CFa) (PKY4=2)(4.E25)
    signDy = sign(Dy);
    signDy(signDy == 0) = 1; % If [Dy = 0] then [sign(0) = 0]. This is done to avoid [Kya / 0 = NaN] in Eqn 4.E26
    By = Kya./(Cy.*Dy + epsilon.*signDy); % (4.E26) [sign(Dy) term explained on page 177]
    SVyg = Fz.*(PVY3 + PVY4.*dfz).*sine_gamma.* LKYC .* LMUY_prime .* zeta2; % (4.E28)
    Kyg0 = Fz.*(PKY6 + PKY7 .*dfz).*(1 + PPY5.*dpi).*LKYC; % (=dFyo./dgamma at alpha = gamma = 0) (= CFgamma) (4.E30)
    signKya = sign(Kya);
    signKya(signKya == 0) = 1; % If [Kya = 0] then [sign(0) = 0]. This is done to avoid [num / 0 = NaN] in Eqn 4.E27
    SHy = (PHY1 + PHY2.*dfz).* LHY + ((Kyg0 .*sine_gamma - SVyg)./(Kya + epsilon.*signKya)).*zeta0 +zeta4 -1; % (4.E27) [sign(Kya) term explained on page 177]
    SVy = Fz.*(PVY1 + PVY2.*dfz).*LVY.*LMUY_prime.*zeta2 + SVyg; % (4.E29)
    alphay = tan(alpha) + SHy; % (4.E20)
    signAlphaY = sign(alphay);
    signAlphaY(signAlphaY == 0) = 1;
    Ey = (PEY1 + PEY2.*dfz).*(1 + PEY5.*sine_gamma.^2 - (PEY3 + PEY4.*sine_gamma).*signAlphaY).*LEY; % (<=1)(4.E24)
    if(any(Ey > 1))
        warning('Solver:CoeffChecks:Ey','Ey over limit (>1), Eqn(4.E24)')
        Ey(Ey > 1) = 1;
    end % if Ey > 1
    Fy0 = Dy.*sin(Cy.*atan(By.*alphay-Ey.*(By.*alphay-atan(By.*alphay))))+ SVy; % (4.E19)

    % Pack pure lateral parameters
    fyOut.Fy0 = Fy0; % (4.E19)
    fyOut.alphay = alphay; % (4.E20)
    fyOut.Cy = Cy; % (4.E21)
    fyOut.Dy = Dy; % (4.E22)
    fyOut.muy = muy; % (4.E23)
    fyOut.Ey = Ey; % (4.E24)
    fyOut.Kya = Kya; % (4.E25)
    fyOut.By = By; % (4.E26)
    fyOut.SHy = SHy; % (4.E27)
    fyOut.SVyg = SVyg; % (4.E28)
    fyOut.SVy = SVy; % (4.E29)
    fyOut.Kyg0 = Kyg0; % (4.E30)
end

function [k_star, Fx0_star, iterGold] = goldenSearchMethod(a, d, tol, maxIter, Fz, gamma, tirParams, proParams)
    % Golden ratio coefficient
    alpha = (sqrt(5) - 1) / 2;
    % Initial interior evaluation points
    b = d - alpha * (d - a);
    c = a + alpha * (d - a);
    for k = 1:maxIter
        % Evaluate magic formula longitudinal force at points b and c
        fx1 = Fx0(Fz, b, gamma, tirParams, proParams);
        fxb = fx1.Fx0;
        fx2 = Fx0(Fz, c, gamma, tirParams, proParams);
        fxc = fx2.Fx0;
        % Termination condition: search window sufficiently small
        if abs(b - a) < tol
            break
        end
        % Maximisation step (choose interval that retains the peak)
        if fxb > fxc
            % Peak lies in [a, c]
            d = c;
            c = b;
            b = d - alpha * (d - a);
        else
            % Peak lies in [b, d]
            a = b;
            b = c;
            c = a + alpha * (d - a);
        end
    end
    % Number of iterations used
    iterGold = k;
    % Select final optimum between the two interior candidates
    if fxb > fxc
        k_star   = b;
        Fx0_star = fxb;
    else
        k_star   = c;
        Fx0_star = fxc;
    end
end

function [speedLim, rpmLim] = calculatePeakAccelDomain(k_star, mass, radius, torqueLim, FDR, momInertia, error)
    speedLim= (torqueLim * radius * error) / (k_star * momInertia * FDR); % speed of limit tire for best accel
    rpmLim = (speedLim*30)/(radius * FDR * pi); % angular velocity limit of motor for best accel
end

function Fnet = calcNetForce(Fx0_star, speedLim, torqueLim, mass, Fz, gamma, vel, radius, FDR, momInertia, tirParams, proParams, error)
    % aerodynamic drag params
    airDensity  = 1.225;     % kg/m^3
    coeffDrag   = 1.543477;  % -
    frontelArea = 1.124197;  % m^2
    Fd = 0.5 * airDensity * coeffDrag * frontelArea * vel^2; % aero drag
    if vel <= speedLim
        % below torque limit → optimal slip
        Fnet = 2*Fx0_star - Fd;
    else
        % torque limit → slip limited by motor torque
        kappaLim = (torqueLim * radius * error) / (FDR * momInertia * vel);
        fxOut  = Fx0(Fz, kappaLim, gamma, tirParams, proParams);
        Fx_lim = fxOut.Fx0;
        Fnet   = 2*Fx_lim - Fd;
    end
end

function dveldt = odeFun(t, vel, Fx0_star, speed_star, torqueLim, mass, Fz, gamma, radius, FDR, momInertia, tirParams, proParams, error)
    Fnet = calcNetForce(Fx0_star, speed_star, torqueLim, mass, Fz, gamma, vel, radius, FDR, momInertia, tirParams, proParams, error);
    dveldt = Fnet/(4*mass); % d(vel)/dt = Fnet/mass (m/s^2)
end

function sol = findV(tMax, Fx0_star, speed_star, torqueLim, mass, Fz, gamma, radius, FDR, momInertia, tirParams, proParams, error)
    vel0 = 0; % initial condition
    tspan = [0 tMax]; % time interval
    sol = ode45(@(t, vel) odeFun(t, vel, Fx0_star, speed_star, torqueLim, mass, Fz, gamma, radius, FDR, momInertia, tirParams, proParams, error), ...
        tspan, vel0);
end

function dist = compSimp38(n, sol, tEnd)
    % Composite Simpson's 3/8 numerical integration
    sigma_t = tEnd/n; % interval size
    delta_t = sigma_t/3; % 3/8 sub-step
    a = 0; b = sigma_t;
    dist = 0;
    for i = 1:n
        term1 = deval(sol,a); % v(t)
        term2 = deval(sol,a+delta_t); % v(t + Δt/3)
        term3 = deval(sol,a+2*delta_t); % v(t + 2Δt/3)
        term4 = deval(sol,b); % v(t + Δt)
        dist = dist + sigma_t/8 * (term1 + 3*term2 + 3*term3 + term4); % accumulate
        a = b; 
        b = b + sigma_t; % shift bounds
    end
end

function [t_star,velEnd,d,i] = newtonRaph(init, sol, maxIter, tol, lengthTarget, tMax, n)
    % Newton-Raphson to solve time such that integrated distance = lengthTarget
    t = init;
    for i = 1:maxIter
        d = compSimp38(n, sol, t) - lengthTarget; % distance error
        grad = deval(sol,t); % velocity(t)
        t = t - d/grad; % Newton update
        if t > tMax, t = tMax; end % clamp to max
        if abs(d) < tol, break; end % convergence
    end
    t_star = t;
    velEnd = deval(sol,t_star); % end speed
end


function testFx0(Fx0_star, k_star, tirParams)
    n = 500;
    alpha = zeros(n,1);
    kappa = linspace(-0.8,0.8,n)';
    gamma = zeros(n,1);
    Pa = ones(n,1)*62052.8;
    Vx = ones(n,1) * 20;
    Fz = ones(n,1).*66*9.81;
    proParams = calculateParams(Fz, Pa, tirParams);
    fxOut = Fx0(Fz, kappa, gamma, tirParams, proParams);

    figure
    plot(kappa, fxOut.Fx0)
    xlabel("slip ratio")
    ylabel("Fx0")
    
    xline(k_star, "-r", "k*", ...
        'LabelOrientation','horizontal',...
        'LabelHorizontalAlignment','center',...
        'LabelVerticalAlignment','middle')
    
    yline(Fx0_star, "-r", "Fx0*", ...
        'LabelOrientation','horizontal',...
        'LabelHorizontalAlignment','center',...
        'LabelVerticalAlignment','middle')


end

function testFy0(tirParams)
    n = 500;
    kappa = zeros(n,1);
    alpha = linspace(-0.85,0.85,n)';
    gamma = zeros(n,1);
    sine_gamma = sin(gamma);
    Pa = ones(n,1)*70000;
    Vx = ones(n,1) * 20;
    Fz = ones(n,1).*500;
    proParams = calculateParams(Fz, Pa, tirParams);
    fyOut = Fy0(Fz, alpha, sine_gamma, tirParams, proParams);

    figure
    plot(alpha, fyOut.Fy0)
    xlabel("slip angle")
    ylabel("Fy0")
end
%% looks like a good thing to take note of

% % Velocities in point S (slip point)
% % Note: why use absolute of longitudinal velocity of the contact patch
% Vsx = -kappa.*abs(Vcx); % [Eqn (4.E5) Page 181 - Book]
% Vsy = tan(alpha).*abs(Vcx); % [Eqn (2.12) Page 67 - Book] and [(4.E3) Page 177 - Book]
% 
% % can we think of the slip velocity as the linear acceleration?
% 
% % Important Note:
% % Due to the ISO sign convention, equation 2.12 does not need a
% % negative sign. The Pacejka book is written in adapted SAE.
% Vs = sqrt(Vsx.^2+Vsy.^2); % [Eqn (3.39) Page 102 - Book] -> Slip velocity of the slip point S
% 
% % Velocities in point C (contact)
% Vcy = Vsy; % Assumption from page 67 of the book, paragraph above Eqn (2.11)
% Vc = sqrt(Vcx.^2+Vcy.^2); % Velocity of the wheel contact centre C, Not described in the book but is the same as [Eqn (3.39) Page 102 - Book]

% [FUNCTIONS FOR TESTING]
fxOut = Fx0(Fz, 0.15, gamma, tirParams, proParams);
% fyOut = Fy0(500, 0.2, 0, tirParams, proParams);
testFx0(tirParams);
% testFy0(tirParams);

data = load("powertrainMotec.mat");
motecDist = data.Vehicle_Odometer_Distance;
motecSpd = data.GPS_Sensor_Speed;
motecSpd.Value = motecSpd.Value / 3.6;
motecTorq = data.Cascadia_Cascadia_Calculated_Torque; % larger by a magnitude of 10
motecTorq.Value = motecTorq.Value/10;

figure

yyaxis("left")
plot(motecSpd.Time, motecSpd.Value)
xlim([525.5 530])
xlabel("Time (s)")
ylabel("Speed (m/s)")

yyaxis("right")
plot(motecTorq.Time, motecTorq.Value)
ylabel("Torque (Nm)")

figure
% Plot the results
t = linspace(0,4.5);
vel = deval(sol,t);
torque = (vel .* FDR .* momInertia .* k_star) ./ (radius .* error);
torque(vel>speedLim) = 230;

yyaxis("left")
plot(t, vel);
xlabel('Time (s)');
ylabel('Velocity (m/s)');

yyaxis("right")
plot(t,torque)
xlabel('Time (s)');
ylabel('Torque (Nm)');

figure
vel = linspace(0,20);
torque = (vel .* FDR .* momInertia .* k_star) ./ (radius .* error);
torque(vel>speedLim) = 230;
plot(vel, torque)
xlabel("Speed (m/s)")
ylabel("Torque (Nm)")