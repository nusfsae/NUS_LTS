function y = smoothPos(x)
k = 50;
y = 0.5 * (1 + tanh(k*x));
end