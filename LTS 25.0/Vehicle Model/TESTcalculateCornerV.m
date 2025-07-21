%this function determines max velocity the tire can give
warning('off','all')
warning

% Tire data visualization and analysis

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
HoosierLC0 = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');
HoosierR20 = mfeval.readTIR('HoosierR20.TIR');
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')

tyre_model = HoosierR20;

% Known variables
mass = 262;
air_density = 1.196;
frontel_area = 1.157757;
CLc = 3.782684;
IA = 0;
phit = 0;
P = 9; % convert psi to pa
useMode = 121;
max_speed = 40;
del_max = rad2deg(0.565);
curv = 8.374;
sen_lat = 1;
sen_long = 1;
tc_long = 0.6077;
tc_lat = 0.6077;
sen_long = 1;
max_rpm = 5500;
FDR = 3.36;
R = 0.2032;
sen_lat = 1;
R1 = 1/0.0934;
R2 = 1/0.0723;
wheelbase = 1.531;
cg_h = 0.256; 
longG = 1;
ab = 0.5310665;
distance = 1;
mass_front = 0.5095; % mass distribution to front
Inertia = 106;

% Find cornering speed
[vel,SA_f] = calculateCornerV(tyre_model,mass,air_density,frontel_area,CLc,R1,R2,IA,max_rpm,FDR,R, ...
    tc_lat,sen_lat,P,wheelbase,del_max,cg_h,longG,ab,distance,mass_front,Inertia);
disp(vel);

function [vel,SA_f] = calculateCornerV(tyre_model,mass,air_density,frontel_area,CLc,R1,R2,IA,max_rpm,FDR,R, ...
    tc_lat,sen_lat,P,wheelbase,del_max,cg_h,longG,ab,distance,mass_front,Inertia)

max_speed = (max_rpm/FDR)*pi*2*R/60;
vel = max_speed;

% ackerman steering angle
del_ack = rad2deg(wheelbase/R); 

SR = 0;

for V = 0.1:0.1:40
    Weight = mass*9.81;
    Downforce = 0.5*air_density*frontel_area*CLc*V^2;
    Fz = Weight+Downforce;

    % amount of load transfer
    delta_W = Fz*longG*cg_h/wheelbase;

    % adjust normal load with load transfer
    Fz_f = (Weight + Downforce*ab)/4 - (delta_W)/2;
    Fz_r = (Weight + Downforce*ab)/4 + (delta_W)/2;
    
    % define the smaller Fz
    Fz_s = Fz_r;
    Fz_b = Fz_f; 

    if Fz_f<Fz_r
        Fz_b = Fz_r;
        Fz_s = Fz_f;
    end
    
    % limit maximum slip angle with max steering angle
    if del_max-del_ack>0
        max_SA = del_max-del_ack;
    else
        max_SA = 0;
    end

    % find SA for lighter loaded tire
    for SA = 0:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_s,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_s,SR,SA+0.1,IA,P,V);

        SA_optimized_s = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_s = SA;            
            break
        end
    end  
    
    % find SA for heavier loaded tire
    for SA = SA_optimized_s:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_b,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_b,SR,SA+0.1,IA,P,V);
        
        SA_optimized_b = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_b = SA;      
            break
        end
    end      
    
    % assign slip angle values to front and rear tires
    if Fz_f<Fz_r
        SA_f = SA_optimized_s;
        SA_r = SA_optimized_b;
    else
        SA_f = SA_optimized_b;
        SA_r = SA_optimized_s;
    end
    
    % limit front slip angle with steering angle
    if SA_f > max_SA
        SA_f = max_SA;
    end

    % find max lateral G and max yaw moment

    % top point: max yaw moment (point 1)
    SA_f1 = SA_f;
    SA_r2 = -SA_r; % rear slip should be negative

    [Fyf1,~] = tires(tyre_model,Fz_f,SR,SA_f1,IA,P,V);
    [Fyr1,~] = tires(tyre_model,Fz_r,SR,SA_r2,IA,P,V);
    Fyf1 = Fyf1*2*sen_lat*tc_lat;
    Fyr1 = Fyr1*2*sen_lat*tc_lat;

    M1 = Fyf1*mass_front*wheelbase-Fyr1*(1-mass_front)*wheelbase;
    G1 = Fyf1+Fyr1;

    % right-most point: max lateral G (point 2)
    SA_f2 = SA_f;
    SA_r2 = SA_r;

    [Fyf2,~] = tires(tyre_model,Fz_f,SR,SA_f2,IA,P,V);
    [Fyr2,~] = tires(tyre_model,Fz_r,SR,SA_r2,IA,P,V);
    Fyf2 = Fyf2*2*sen_lat*tc_lat;
    Fyr2 = Fyr2*2*sen_lat*tc_lat;

    M2 = Fyf2*mass_front*wheelbase-Fyr2*(1-mass_front)*wheelbase;
    G2 = Fyf2+Fyr2;
    
    % bottom point: min yaw moment (negative max) (point 3)
    SA_f3 = -SA_f;
    SA_r3 = SA_r;
    
    [Fyf3,~] = tires(tyre_model,Fz_f,SR,SA_f3,IA,P,V);
    [Fyr3,~] = tires(tyre_model,Fz_r,SR,SA_r3,IA,P,V);
    Fyf3 = Fyf3*2*sen_lat*tc_lat;
    Fyr3 = Fyr3*2*sen_lat*tc_lat;
    
    M3 = Fyf3*mass_front*wheelbase-Fyr3*(1-mass_front)*wheelbase;
    G3 = Fyf3+Fyr3;
    
    % left-most point: min lateral G (negative max) (point 4)
    SA_f4 = -SA_f;
    SA_r4 = -SA_r;
    
    [Fyf4,~] = tires(tyre_model,Fz_f,SR,SA_f4,IA,P,V);
    [Fyr4,~] = tires(tyre_model,Fz_r,SR,SA_r4,IA,P,V);
    Fyf4 = Fyf4*2*sen_lat*tc_lat;
    Fyr4 = Fyr4*2*sen_lat*tc_lat;
    
    M4 = Fyf4*mass_front*wheelbase-Fyr4*(1-mass_front)*wheelbase;
    G4 = Fyf4+Fyr4;

    % Generate graph of YMD
    % gradient
    grad12 = (M1-M2)/(G1-G2);
    grad23 = (M2-M3)/(G2-G3);
    grad34 = (M3-M4)/(G3-G4);
    grad41 = (M4-M1)/(G4-G1);
    % intercept
    b12 = M2 - grad12*G2;
    b23 = M3 - grad23*G3;
    b34 = M4 - grad34*G4;
    b41 = M1 - grad41*G1;

    % Find required yaw moment to turn
    Mreq = Inertia * (V^2*(1/R2 - 1/R1))/distance;

    % Find required cornering force to turn
    FYreq = (mass*V^2)/R1; 

    % Check whether have enough yaw momemnt
    if Mreq >= M4 && Mreq <= M2
        Lat = findYMD(Mreq,grad12,grad23,grad34,grad41,b12,b23,b34,b41,R1);
        % Check if required force to corner higher than max force tyre can provide
        if Lat < FYreq
            vel = V-0.1;
            break
        end
    else
        vel = V-0.1;
    end

end

del = del_ack + SA_f;

end


function G = findYMD(Mreq,grad12,grad23,grad34,grad41,b12,b23,b34,b41,R1)

if Mreq > 0 && R1 > 0
    G = grad12*Mreq + b12;
elseif Mreq > 0 && R1 < 0
    G = grad41*Mreq + b41;    
elseif Mreq < 0 && R1 > 0
    G = grad23*Mreq + b23;    
elseif Mreq < 0 && R1 < 0
    G = grad34*Mreq + b34;
end
end
