% this function finds previous point max speed under braking scenario
% 
% given velocity of current point, find max speed of the previous point
% v2 is speed of current point, v1 is upper limit of previous point speed
% (v1>v2)

for v0 = v2:0.1:v1
    % current lateral acceleration
    ay =v0^2/radius;
    % available longitudinal acceleraton
    ax =findax(v0,ay);
    % minimum speed after decceleration
    v =sqrt(v0^2+2*ax*distance);
    % break if minimum speed less than target speed
    if v<=v2
        break
    end
end