function outputs = vehicleModel(inputs,carData)

    % Constants
    g = 9.81;
    rho = 1.225;
    
    % Aerodynamic Calculations
    DF_total = 0.5*1.225*carData.Aero.ClA*inputs.Vx^2;
    DF_front = carData.Aero.rAeroBalance*DF_total;
    DF_rear = (1-carData.Aero.rAeroBalance)*DF_front;
    Fd = 0.5*rho*carData.Aero.CdA*inputs.Vx^2;
    
    % Slip Angles
    alpha_fl = ((inputs.Vx*tan(inputs.beta) + carData.Chassis.frontMomentArm*inputs.yawRate) / (inputs.Vx + inputs.yawRate*carData.Chassis.trackWidth*0.5)) - (inputs.delta - carData.Suspension.aToeStaticFront*pi/180); % [deg], negative is toe inwards, wheel pointing inwards to chassis
    alpha_fr = ((inputs.Vx*tan(inputs.beta) + carData.Chassis.frontMomentArm*inputs.yawRate) / (inputs.Vx - inputs.yawRate*carData.Chassis.trackWidth*0.5)) - (inputs.delta + carData.Suspension.aToeStaticFront*pi/180);
    alpha_rl = ((inputs.Vx*tan(inputs.beta) - carData.Chassis.rearMomentArm*inputs.yawRate) / (inputs.Vx + inputs.yawRate*carData.Chassis.trackWidth*0.5))  - (-carData.Suspension.aToeStaticRear*pi/180); % [deg], negative is toe inwards, wheel pointing inwards to chassis
    alpha_rr = ((inputs.Vx*tan(inputs.beta) - carData.Chassis.rearMomentArm*inputs.yawRate) / (inputs.Vx - inputs.yawRate*carData.Chassis.trackWidth*0.5))  - (carData.Suspension.aToeStaticRear*pi/180);
    
    % Wheel Velocities
    wheel_rot_fl = (inputs.kappa_fl + 1)*inputs.Vx/carData.Chassis.radWheel;
    wheel_rot_fr = (inputs.kappa_fr + 1)*inputs.Vx/carData.Chassis.radWheel;
    wheel_rot_rl = (inputs.kappa_rl + 1)*inputs.Vx/carData.Chassis.radWheel;
    wheel_rot_rr = (inputs.kappa_rr + 1)*inputs.Vx/carData.Chassis.radWheel;
    
    % Lateral Load Transfer
    del_w_f = (carData.Chassis.SprungMass * inputs.ay_control * carData.Suspension.heightCG2rollAxis * carData.Suspension.mechanicalBalance/carData.Chassis.trackWidth)...
        + (carData.Chassis.sprungMassFront *inputs.ay_control * carData.Suspension.rollCentreFront / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * inputs.ay_control / carData.Chassis.trackWidth);
    del_w_r = (carData.Chassis.SprungMass * inputs.ay_control * carData.Suspension.heightCG2rollAxis * (1-carData.Suspension.mechanicalBalance)/carData.Chassis.trackWidth)...
        + (carData.Chassis.sprungMassRear * inputs.ay_control * carData.Suspension.rollCentreRear / carData.Chassis.trackWidth) + (carData.Chassis.unsprungMass * carData.Chassis.heightUnsprungCOG * inputs.ay_control / carData.Chassis.trackWidth);
    
    % Longitudinal Load Transfer
    longLT = carData.Chassis.mass * inputs.ax_control * carData.Chassis.heightSprungCOG / (2 * carData.Chassis.wheelBase);
    
    % Wheel Loads 
    w_fl = (carData.Chassis.massFront * g / 2) - (del_w_f) - longLT + (DF_front/2);
    w_fr = (carData.Chassis.massFront * g / 2) + (del_w_f) - longLT + (DF_front/2);
    w_rl = (carData.Chassis.massRear * g / 2) - (del_w_r) + longLT + (DF_rear/2);
    w_rr = (carData.Chassis.massRear * g / 2) + (del_w_r) + longLT + (DF_rear/2);
    
    % Wheel Forces
    [fy_fl, fx_fl] = Tyres.MF52(inputs.kappa_fl,alpha_fl,w_fl,carData.Suspension.aCamberFront, carData);
    [fy_fr, fx_fr] = Tyres.MF52(inputs.kappa_fr,alpha_fr,w_fr,-carData.Suspension.aCamberFront, carData);
    [fy_rl, fx_rl] = Tyres.MF52(inputs.kappa_rl,alpha_rl,w_rl,carData.Suspension.aCamberRear, carData);
    [fy_rr, fx_rr] = Tyres.MF52(inputs.kappa_rr,alpha_rr,w_rr,-carData.Suspension.aCamberRear, carData);
    
    brakeBias_tyre = (fx_fl + fx_fr) / (fx_fl + fx_fr + fx_rl + fx_rr);
    power_out = ( fx_fl + fx_fr + fx_rl + fx_rr ) * inputs.Vx / 1000; % Wheel Power in kW
    
    % Car States
    ay_out = (fy_fl + fy_fr + fy_rl + fy_rr)/carData.Chassis.mass;
    ax_out = (fx_fl + fx_fr + fx_rl + fx_rr - Fd)/carData.Chassis.mass;
    Mz_out = (carData.Chassis.frontMomentArm*(fy_fl + fy_fr) + 0.5*carData.Chassis.trackWidth*(fx_fl + fx_rl) ...
              - carData.Chassis.rearMomentArm*(fy_rl + fy_rr) - 0.5*carData.Chassis.trackWidth*(fx_fr + fx_rr))/carData.Chassis.yawInertia;
    
    
    outputs = struct;
    outputs.ay = ay_out;
    outputs.ax = ax_out;
    outputs.Mz = Mz_out;
    outputs.omegaWheelFL = wheel_rot_fl;
    outputs.omegaWheelFR = wheel_rot_fr;
    outputs.omegaWheelRL = wheel_rot_rl;
    outputs.omegaWheelRR = wheel_rot_rr;
    outputs.aSlipFL = alpha_fl;
    outputs.aSlipFR = alpha_fr;
    outputs.aSlipRL = alpha_rl;
    outputs.aSlipRR = alpha_rr;
    outputs.powerWheel = power_out;

    % Residuals
    outputs.residuals.ax_res = inputs.ax_control - ax_out;
    outputs.residuals.ay_res = inputs.ay_control - ay_out;
    outputs.residuals.brakeBias_res = carData.Brakes.rBrakeBias - brakeBias_tyre;
    outputs.residuals.power = power_out - carData.Powertrain.MaxPower;

end