% Smoothen mininum function for linearity
% x,y:variables to compare
% alpha: lower value for accuracy
function ymin = smoothmin(x,y,alpha)

% works when x,y are close
% ymin =0.5*(x + y) - (1/(2*alpha)) * log(cosh(alpha*(x - y)));

% unstable when x,y are not close
% ymin =x - (1/alpha) * log(1 + exp(-alpha*(y - x)));

% numerical unstable when x,y are large
% ymin = -(1/alpha) * log(exp(-alpha*x) + exp(-alpha*y));

% this works the best
ymin =(x + y - sqrt((y-x).^2 + alpha))./2;
end
