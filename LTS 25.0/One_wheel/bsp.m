%determine boundary speed profile for the car
%corner speed considered with effects of aero
%corner speed calculation base on LTS (James,2000) pg29

function BSP = bsp(mass,C2,air_density,frontel_area,CLc,tyre_model,camber,max_rpm,FDR,R,tc_lat,sen_lat)

num = length(C2); %number of points on track
BSP = zeros(num,1); %create zero array
slip_ang = slip_angle(C2); %obtain slip angle list for this given track
latG = zeros(num,1);

for point = 1:num
    curv = abs(1/C2(point));%radius of curvature at current point
    alpha = slip_ang(point);%slip angle of this point
   
    if abs(curv) > 30 %radius too large, track is straight
        
        BSP(point) = (max_rpm/FDR)*pi*2*R/60;
        latG(point) = 0;
        
    else %cornering scenario

        %find max corner speed using 'v_calculater' function
        BSP(point) = v_calculater(tyre_model,mass,air_density,frontel_area,CLc,curv,camber,alpha,max_rpm,FDR,R,tc_lat,sen_lat);
        latG(point) = (BSP(point)^2/curv)/9.81;     
    end

end

function 