function [kWh_Usage, X, Y, norm_torque, norm_throttle, norm_speed] = EnergyConsumption(Torque, Speed, dist, Throttle, LapseTime, Corr_Speed, Brake_Pressure_Front, Motor_Torque__Updated_, BMS_Channels_Pack_Current, Throttle_Pedal, Power)

%%TRAINING

%Initiating Lists for training data
train_speed = [];
train_torque = [];
train_Current = [];
train_throttle = [];
train_power = [];

% Data filtering 
% Motec Channels might differ according to data exported (Check Channel
% name and update accordingly
for i = 1:length(Corr_Speed.Value)
    x = Brake_Pressure_Front.Value(i);
    % Remove Brake Data using if Statement
    if x >= 0.2
        continue
    else
        train_speed = [train_speed; Corr_Speed.Value(1, i)];
        train_torque = [train_torque; Motor_Torque__Updated_.Value(1, i)];
        train_Current = [train_Current; BMS_Channels_Pack_Current.Value(1, i)]; 
        train_throttle = [train_throttle; Throttle_Pedal.Value(1, i)];
        train_power = [train_power; Power.Value(1, i)];
    end    
end    

% Normalize features using min-max normalization
min_speed = min(train_speed);
max_speed = max(train_speed);
norm_speed = (train_speed - min_speed) / (max_speed - min_speed);

min_torque = min(train_torque);
max_torque = max(train_torque);
norm_torque = (train_torque - min_torque) / (max_torque - min_torque);

min_throttle = min(train_throttle);
max_throttle = max(train_throttle);
norm_throttle = (train_throttle - min_throttle) / (max_throttle - min_throttle);

% Prepare normalized features
X = [norm_torque, norm_throttle, norm_speed];

%X = [train_torque, train_throttle, train_speed];
Y = train_power;

% Define CV partition for hold-out method
cv = cvpartition(length(Y), Holdout = 0.3);
X_train = X(training(cv), :);
X_test = X(test(cv), :);
y_train = Y(training(cv));
y_test = Y(test(cv));

% Initialize variables to store R-squared and RMSE values for each degree
R_squared_values = zeros(5, 1);
RMSE_values = zeros(5, 1);
best_fit = struct('Degree', 0, 'Model', [], 'R_squared', 0, 'RMSE', Inf);

% Loop through polynomial degrees 1 to 5
for degree = 1:5
    % Generate polynomial terms for the current degree using training data
    polyTerms = polyfitn(X_train, y_train, degree);
    
    % Predict on the test set
    y_pred = polyvaln(polyTerms, X_test);
    
    % Calculate R-squared and RMSE
    SS_res = sum((y_test - y_pred).^2);
    SS_tot = sum((y_test - mean(y_test)).^2);
    R_squared = 1 - (SS_res / SS_tot);
    RMSE = sqrt(mean((y_test - y_pred).^2));
    
    % Store metrics for this degree
    R_squared_values(degree) = R_squared;
    RMSE_values(degree) = RMSE;
       
    % Update best fit if the model improves more than 1 percent
    if abs(best_fit.R_squared-R_squared) >= 0.01 % if the change in error is greater than 1%
        best_fit.Degree = degree;
        best_fit.Model = polyTerms;
        best_fit.R_squared = R_squared;
        best_fit.RMSE = RMSE;
    end
end

% Display results
disp('R-squared values for each polynomial degree:');
disp(R_squared_values);
disp('RMSE values for each polynomial degree:');
disp(RMSE_values);
disp('Best fit model:');
disp(best_fit);

% Initiating y = x line
x = linspace(0,40,41);
y = x;

% Plot predicted vs. actual current draw for the best model
predicted_current = polyvaln(best_fit.Model, X_test);
figure;
scatter(y_test, predicted_current);
hold on
plot(x,y,'r-'); 
xlabel('Actual Power Draw');
ylabel('Predicted Power Draw');
title(['Predicted vs. Actual Power Draw - Best Model (Degree ', num2str(best_fit.Degree), ')']);
grid on;


%%TESTING

% Using LTS outputs as inputs for prediction
Throttle =  Throttle*100;
Speed = transpose(Speed);
X_lts1 = [Torque, Throttle, Speed]; 
filtered_X_lts = X_lts1(X_lts1(:, 2) > 0, :);
min_ThrottleLTS = min(filtered_X_lts(:,1));
max_ThrottleLTS = max(filtered_X_lts(:,1));
norm_ThrottleLTS = (filtered_X_lts(:,1)- min_ThrottleLTS)/(max_ThrottleLTS-min_ThrottleLTS);

min_SpeedLTS = min(filtered_X_lts(:,3));
max_SpeedLTS = max(filtered_X_lts(:,3));
norm_SpeedLTS = (filtered_X_lts(:,3)-min_SpeedLTS)/(max_SpeedLTS-min_SpeedLTS);

min_TorqueLTS = min(filtered_X_lts(:,2));
max_TorqueLTS = max(filtered_X_lts(:,2));
norm_TorqueLTS = (filtered_X_lts(:,2)-min_TorqueLTS)/(max_TorqueLTS-min_TorqueLTS);

X_lts = [norm_TorqueLTS, norm_ThrottleLTS, norm_SpeedLTS];

kWh_Usage1 = zeros(length(dist),1);
power_map = zeros(length(dist),1);
power_prediction = polyvaln(polyTerms,X_lts);
Count = 0;
for i = 1: length(dist)-1
    Testing = Torque(i);
    if Torque(i) > 0
        Count = Count +1;
        power_map(i) = power_prediction(Count);
    else
        power_map(i) = 0;
    end
end
for i = 1: length(dist)-1
    kWh_Usage1(i)= power_map(i)*(((LapseTime(i+1)-LapseTime(i))/3600)); 
end

figure;
plot(LapseTime,power_map,'r-');
xlabel('Lapsed Time (s)')
ylabel('Power (kW)')
title('Power Output vs Lapsed Time')
hold on

%load('Lap 1 Endurance.mat')
%plot(Power.Time-132.178,Power.Value, 'b-')

Power_usage = sum(Power.Value);

kWh_Usage = sum(kWh_Usage1);

TOTAL = kWh_Usage;
