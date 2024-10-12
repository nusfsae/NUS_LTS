%this function determines slip angle based on geometry
%assumption explanation is in doc
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

%len = length(pos.x); %length of track
%slip_ang = zeros(1,len);

%slip_ang(1,len-1) = 0;
%slip_ang(1,len) = 0;

%for point = 1:len-2
 %   x1 = pos.x(point); %coordinates of three continuous points
  %  y1 = pos.y(point);
   % x2 = pos.x(point+1);
    %y3 = pos.x(point+2);
    %y3 = pos.y(point+2);

   % side12 = sqrt((x2-x1)^2+(y2-y1)^2);
    %side23 = sqrt((x3-x2)^2+(y3-y2)^2);
    %side13 = sqrt((x3-x1)^2+(y3-y1)^2);

    %slip_ang(1,point) = acosd((side12^2+side13^2-side23^2)/(2*side12*side13)); %cosine rule