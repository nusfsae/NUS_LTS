% % convert steering data
% % pro-ackermann assumption (inner wheel steer more)
% 
% function [in,out] = ackerman(mid,source)
% ack_percentage = source(10:end,3);
% delta_inner = source(10:end,4);
% delta_outer = source(10:end,5);
% delta_ratio = source(10:end,6);
% delta_mid = (delta_inner+delta_outer)/2;
% 
% % Maximum polynomial degree to test
% max_degree = 6;
% 
% % Initialize arrays to store metrics
% degrees = 1:max_degree;
% rmse = zeros(1, max_degree);
% r_squared = zeros(1, max_degree);
% aic = zeros(1, max_degree);
% 
% % Calculate metrics for each polynomial degree
% for deg = degrees
%     % Fit polynomial
%     p = polyfit(delta_mid, delta_outer, deg);
%     y_fit = polyval(p, delta_mid);
% 
%     % Calculate RMSE (Root Mean Square Error)
%     rmse(deg) = sqrt(mean((delta_outer - y_fit).^2));
% 
%     % Calculate R-squared
%     ss_res = sum((delta_outer - y_fit).^2);
%     ss_tot = sum((delta_outer - mean(delta_outer)).^2);
%     r_squared(deg) = 1 - (ss_res / ss_tot);
% 
%     % Calculate AIC (Akaike Information Criterion)
%     n = length(delta_outer);
%     k = deg + 1; % number of parameters
%     aic(deg) = n * log(ss_res/n) + 2*k;
% end
% 
% % Find best degree using AIC (lower is better)
% [~, best_deg_aic] = min(aic);
% 
% % Use AIC as primary criterion
% best_degree = best_deg_aic;
% 
% % Fit best polynomial
% p_best = polyfit(delta_mid, delta_outer, best_degree);
% out = polyval(p_best, mid);
% 
% %% inner
% % Initialize arrays to store metrics
% degrees = 1:max_degree;
% rmse = zeros(1, max_degree);
% r_squared = zeros(1, max_degree);
% aic = zeros(1, max_degree);
% 
% % Calculate metrics for each polynomial degree
% for deg = degrees
%     % Fit polynomial
%     p = polyfit(delta_mid, delta_inner, deg);
%     y_fit = polyval(p, delta_mid);
% 
%     % Calculate RMSE (Root Mean Square Error)
%     rmse(deg) = sqrt(mean((delta_inner - y_fit).^2));
% 
%     % Calculate R-squared
%     ss_res = sum((delta_inner - y_fit).^2);
%     ss_tot = sum((delta_inner - mean(delta_inner)).^2);
%     r_squared(deg) = 1 - (ss_res / ss_tot);
% 
%     % Calculate AIC (Akaike Information Criterion)
%     n = length(delta_inner);
%     k = deg + 1; % number of parameters
%     aic(deg) = n * log(ss_res/n) + 2*k;
% end
% 
% % Find best degree using AIC (lower is better)
% [~, best_deg_aic] = min(aic);
% 
% % Use AIC as primary criterion
% best_degree = best_deg_aic;
% 
% % Fit best polynomial
% p_best = polyfit(delta_mid, delta_inner, best_degree);
% in = polyval(p_best, mid);
% 
% end


% convert steering data
% pro-ackermann assumption (inner wheel steer more)
function [in, out] = ackerman(mid, source)
    ack_percentage = source(10:end,3);
    delta_inner = source(10:end,4);
    delta_outer = source(10:end,5);
    delta_ratio = source(10:end,6);
    delta_mid = (delta_inner + delta_outer) / 2;
    
    % Maximum polynomial degree to test
    max_degree = 6;
    
    %% Fit outer wheel polynomial
    degrees = 1:max_degree;
    aic = zeros(1, max_degree);
    
    % Calculate metrics for each polynomial degree
    for deg = degrees
        % Fit polynomial
        p = polyfit(delta_mid, delta_outer, deg);
        y_fit = polyval(p, delta_mid);
        
        % Calculate AIC (Akaike Information Criterion)
        ss_res = sum((delta_outer - y_fit).^2);
        n = length(delta_outer);
        k = deg + 1; % number of parameters
        aic(deg) = n * log(ss_res/n) + 2*k;
    end
    
    % Find best degree using AIC (lower is better)
    [~, best_deg_aic] = min(aic);
    best_degree = best_deg_aic;
    
    % Fit best polynomial and get coefficients
    p_outer = polyfit(delta_mid, delta_outer, best_degree);
    
    % Evaluate using manual polynomial evaluation (CasADi compatible)
    out = poly_eval(p_outer, mid);
    
    %% Fit inner wheel polynomial
    aic = zeros(1, max_degree);
    
    % Calculate metrics for each polynomial degree
    for deg = degrees
        % Fit polynomial
        p = polyfit(delta_mid, delta_inner, deg);
        y_fit = polyval(p, delta_mid);
        
        % Calculate AIC (Akaike Information Criterion)
        ss_res = sum((delta_inner - y_fit).^2);
        n = length(delta_inner);
        k = deg + 1; % number of parameters
        aic(deg) = n * log(ss_res/n) + 2*k;
    end
    
    % Find best degree using AIC (lower is better)
    [~, best_deg_aic] = min(aic);
    best_degree = best_deg_aic;
    
    % Fit best polynomial and get coefficients
    p_inner = polyfit(delta_mid, delta_inner, best_degree);
    
    % Evaluate using manual polynomial evaluation (CasADi compatible)
    in = poly_eval(p_inner, mid);
end

% Helper function: Manual polynomial evaluation compatible with CasADi
function y = poly_eval(coeffs, x)
    % Evaluates polynomial with coefficients in descending order
    % y = coeffs(1)*x^n + coeffs(2)*x^(n-1) + ... + coeffs(end)
    n = length(coeffs);
    y = coeffs(1);
    for i = 2:n
        y = y * x + coeffs(i);
    end
end