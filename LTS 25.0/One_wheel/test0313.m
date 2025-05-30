HoosierLC0 = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');

% Known variables
mass = 274;
air_density = 1.196;
frontel_area = 1.157757;
CLc = 3.782684;
camber = 0;
phit = 0;
P = convpres(10, 'psi', 'Pa'); % convert psi to pa
useMode = 121;
tyre_model = HoosierLC0;
max_speed = 40;
max_steer = 0.565;
curv = 8.374;
tc_lat = 0.67;
sen_lat = 1;
tc_long = 0.67;
sen_long = 1;



f = [2,0.05,0.2,0.05,0.2,0.2];
[Fyf,Fxf] = findFf(mass,air_density,frontel_area,CLc,f,camber,phit,P,tyre_model,useMode,tc_lat,sen_lat,tc_long,sen_long);
[Fyr,Fxr] = findFr(mass,air_density,frontel_area,CLc,f,camber,phit,P,tyre_model,useMode,tc_lat,sen_lat,tc_long,sen_long);


% f(1): V  f(2): SR_f  f(3): alpha_f  f(4): SR_r  f(5): alpha_r  f(6): delta

% minimize fun, must be positive


fun = @(f) Fxf*sin(f(6))+Fyf*cos(f(6))+Fyr-((mass*f(1)^2)/curv);

Fc = Fxf*sin(f(6))+Fyf*cos(f(6))+Fyr;
disp(Fxf);

[Fyf,Fxf] = findFf(mass,air_density,frontel_area,CLc,f,camber,phit,P,tyre_model,useMode,tc_lat,sen_lat,tc_long,sen_long);
[Fyr,Fxr] = findFr(mass,air_density,frontel_area,CLc,f,camber,phit,P,tyre_model,useMode,tc_lat,sen_lat,tc_long,sen_long);

nonlcon = @(f) deal(-fun(f),[]); % negative of fun must be negative, so fun must be positive

lb = [0,0,0,0,0,0];
ub = [20,0.2,f(6),0.2,f(6),max_steer];

f0 = [9,0.05,0.2,0.05,0.2,0.2]; % initial guess

A = [];
b = [];
Aeq = [];
Beq = [];

options = optimoptions('fmincon', ...
    'Algorithm', 'interior-point', ...
    'MaxIterations', 50000, ...
    'MaxFunctionEvaluations', 10000, ...
    'OptimalityTolerance', 1e-9, ...
    'StepTolerance', 1e-9, ...
    'ConstraintTolerance', 1e-9, ...
    'Display', 'off');

f = fmincon(fun,f0,A,b,Aeq,Beq,lb,ub,nonlcon,options);

disp(f);

% resolve resultant tire force
% Fcorner = Fxf*sin(delta) + Fyf*cos(delta) + Fyr; % delta: steering angle
% Ftractive = Fxf*cos(delta) - Fyf*sin(delta) + Fxr;
% 
% Fsum = mass*V^2/curv;



function [Fyf,Fxf] = findFf(mass,air_density,frontel_area,CLc,f,camber,phit,P,tyre_model,useMode,tc_lat,sen_lat,tc_long,sen_long)

% Tire output for front tires
load_f = 0.25*(mass*9.81 + 0.5*air_density*frontel_area*CLc*(f(1)^2)); %per tire

input_f = [load_f f(2) f(3) camber phit f(1) P];
[ output_f ] = mfeval(tyre_model, input_f, useMode);

Fyf = 2*tc_lat*sen_lat*abs(output_f(2));  % two tires at the front
Fxf = 2*tc_long*sen_long*abs(output_f(1));

end

function [Fyr,Fxr] = findFr(mass,air_density,frontel_area,CLc,f,camber,phit,P,tyre_model,useMode,tc_lat,sen_lat,tc_long,sen_long)
% Tire output for rear tires
load_r = 0.25*(mass*9.81 + 0.5*air_density*frontel_area*CLc*(f(1)^2)); %per tire

input_r = [load_r f(4) f(3) camber phit f(1) P];
[ output_r ] = mfeval(tyre_model, input_r, useMode);

Fyr = 2*tc_lat*sen_lat*abs(output_r(2)); 
Fxr = 2*tc_long*sen_long*abs(output_r(1));
end