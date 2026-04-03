
% [VEHICLE / TYRE PARAMETERS]
radius = 7.625+1.5; % m
speed = 20; % m/s
mass = (199.6+65)/4; % kg, mass on one tire
g = 9.81; % m/s^2
kappa = 0; % dimensionless
gamma = 0; % rad
pres = 9 * 6894.76; % Pa

% [FIND OPTIMAL SLIP RATIO (max Fx0)]
a       = 0.05;
d       = 0.45;
tol     = 1e-7;
maxIter = 5000;

% aero load
den = 1.196;                         % air density (kgm^-3)
farea = 1.157757;                    % frontel area (m^2)
CL = 3.828;                         % CL cornering
CD = 1.403;                         % CD cornering

Lift = 0.5*den*(speed^2)*CL*farea;

Fzfr = mass * g;
Fzfl = mass * g;
Fzrr = mass * g;
Fzrl = mass * g;

Fcpe = centripetalForce(radius, speed, mass);
[~, Flatfr, ~] = goldenSearchMethod(a, d, tol, maxIter, Fzfr, kappa, gamma, pres);
[~, Flatfl, ~] = goldenSearchMethod(a, d, tol, maxIter, Fzfl, kappa, gamma, pres);
[~, Flatrr, ~] = goldenSearchMethod(a, d, tol, maxIter, Fzrr, kappa, gamma, pres);
[~, Flatrl, ~] = goldenSearchMethod(a, d, tol, maxIter, Fzrl, kappa, gamma, pres);

Flatnet = (Flatfr + Flatfl + Flatrr + Flatrl)+Lift;

muf = frictioncoeff(Flatnet, Fcpe);
disp(muf)
disp(Fcpe)
disp(Flatnet)


function [a_star, Fy0_star, iterGold] = goldenSearchMethod(a, d, tol, maxIter, Fz, kappa, gamma, pres)
    % Golden ratio coefficient
    alpha = (sqrt(5) - 1) / 2;
    % Initial interior evaluation points
    b = d - alpha * (d - a);
    c = a + alpha * (d - a);
    for k = 1:maxIter
        % Evaluate magic formula longitudinal force at points b and c
        [~, fyb] = SM_MF61(Fz, kappa, b, gamma, pres);
        [~, fyc] = SM_MF61(Fz, kappa, c, gamma, pres);
        % Termination condition: search window sufficiently small
        if abs(b - a) < tol
            break
        end
        % Maximisation step (choose interval that retains the peak)
        if fyb > fyc
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
    if fyb > fyc
        a_star   = b;
        Fy0_star = fyb;
    else
        a_star   = c;
        Fy0_star = fyc;
    end
end

function  Fcpe = centripetalForce(radius, speed, mass)
    Fcpe = mass * speed^2 / radius;
end

function muf = frictioncoeff(Flatnet, Fcpe)
     muf = Fcpe / Flatnet;
end
