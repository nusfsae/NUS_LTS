% currently no model for slip angle
% this assumes straight line SA = 1 deg, corner SA = 10 deg
%this output a row matrix of slip angle for a specific track

function slip_ang = slip_angle(C2)

num = length(C2); %number of points on track
slip_ang = zeros(num,2); %create zero array

for point = 1:num
    curv = abs(1/C2(point));%radius of curvature at current point
    slip_ang(point,2) = point;
    
    if curv >50
        slip_ang(point,1) = 1;

    else
        slip_ang(point,1) = 10;
    end


end

end

