function [kWh_Usage, power_map] = predictEnergyUsage(Torque, Speed, Throttle, dist, LapseTime)

    % Load model and normalization params
    persistent modelParams
    if isempty(modelParams)
        modelParams = load("D:\FSAEMain\LTS 25.0\Powertrain\EnergyModel.mat");
    end

    % Apply same scaling to new inputs
    Throttle = Throttle * 100;
    Speed = Speed(:)';
    X_input = [Torque(:), Throttle(:), Speed(:)];
    X_input = X_input(X_input(:,2) > 0, :);

    % Min-max normalize using training params
    norm_Torque = (X_input(:,1) - modelParams.min_torque) / ...
                  (modelParams.max_torque - modelParams.min_torque);
    norm_Throttle = (X_input(:,2) - modelParams.min_throttle) / ...
                    (modelParams.max_throttle - modelParams.min_throttle);
    norm_Speed = (X_input(:,3) - modelParams.min_speed) / ...
                 (modelParams.max_speed - modelParams.min_speed);

    X_test = [norm_Torque, norm_Throttle, norm_Speed];
    power_prediction = polyvaln(modelParams.best_fit.Model, X_test);

    % Match back predictions to full dist size
    kWh_Usage1 = zeros(length(dist),1);
    power_map = zeros(length(dist),1);
    Count = 0;
    for i = 1:length(dist)-1
        if Torque(i) > 0
            Count = Count + 1;
            power_map(i) = power_prediction(Count);
        else
            power_map(i) = 0;
        end
    end

    for i = 1:length(dist)-1
        kWh_Usage1(i) = power_map(i) * ((LapseTime(i+1) - LapseTime(i)) / 3600);
    end

    kWh_Usage = sum(kWh_Usage1);

