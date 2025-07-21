%determine boundary speed profile for the car
%corner speed considered with effects of aero
%corner speed calculation base on LTS (James,2000) pg29

function [sim] = cornerProfile(mass,C2,air_density,frontel_area,CLc,tyre_model,camber,max_rpm,FDR,R,tc_lat,sen_lat,phit,P,wheelbase,del_max,cg_h,Long_Accel,ab,sim)

num = length(C2); %number of points on track


for point = 1:num  
    curv = abs(1/C2(point));
   
    if abs(curv) > 30 %radius too large, track is straight
        
        sim.speed(point,1) = (max_rpm/FDR)*pi*2*R/60;
        sim.latG(point,1) = 0;
        
    else %cornering scenario

        longG = Long_Accel(point);

        %find max corner speed using 'v_calculater' function
        [sim.speed(point,1)] = calculateCornerV(tyre_model,mass,air_density,frontel_area,CLc,curv,camber, ...
            max_rpm,FDR,R,tc_lat,sen_lat,phit,P,wheelbase,del_max,cg_h,longG,ab);
        sim.latG(point,1) = (sim.speed(point,1)^2/curv)/9.81;     
    end

end
