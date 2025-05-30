% Use this as a testing platform for whatever needs

warning('off','all')
warning

% Tire data visualization and analysis

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
HoosierLC0 = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');
HoosierR20 = mfeval.readTIR('HoosierR20.TIR');
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')


% Known variables
mass = 262;
air_density = 1.196;
frontel_area = 1.157757;
CLc = 3.782684;
camber = 0;
phit = 0;
P = 9; % convert psi to pa
useMode = 121;
max_speed = 40;
del_max = rad2deg(0.565);
curv = 8.374;
sen_lat = 1;
tc_long = 0.6077;
sen_long = 1;
max_rpm = 5500;
FDR = 3.36;
R = 0.2032;
sen_lat = 1;


tyre_model = HoosierR20;

SR = 0;
IA = 0;


C2 = 1/4.0338; %8.275; % turn radius 4.0338 tightest turn in JTC
wheelbase = 1.528;

R = 1/C2;
del_ack = rad2deg(wheelbase/R); % ackerman steering angle

% what's new?
cg_h = 0.256; % cg height
Fx = 1000; % assume tractive force is 1000N
ab = 0.55; % assume aero balance 0.55 front
tc_lat = 0.6; % new tcf with KJ constant radius test
del_max = (29.611+26.005)/2;
track = 1.21;

longG = -0.4; % obtain from Run1 with one-wheel model
latG = -1.3;

% longG = Long_Accel(point);
% latG = Lat_Accel(point);

for V = 0.1:0.1:40
    Weight = mass*9.81;
    Downforce = 0.5*air_density*frontel_area*CLc*V^2;
    Fz = Weight+Downforce;

    % amount of load transfer
    delta_Wx = Fz*longG*cg_h/wheelbase;
    delta_Wy = Fz*latG*cg_h/track;

    % adjust normal load with load transfer
    Fz_fr = (Weight + Downforce*ab)/4 - (delta_Wx)/2 + (delta_Wy)/2; % Front Right
    Fz_rr = (Weight + Downforce*ab)/4 + (delta_Wx)/2 + (delta_Wy)/2; % Rear Right
    Fz_fl = (Weight + Downforce*ab)/4 - (delta_Wx)/2 - (delta_Wy)/2; % Front Left
    Fz_rl = (Weight + Downforce*ab)/4 + (delta_Wx)/2 - (delta_Wy)/2; % Rear Left    


    % % FRONT TIRES
   
    % define the smaller and bigger Fz for front tires
    rank_Fzf = sort([Fz_fl,Fz_fr]);
    Fz_fs = rank_Fzf(1);
    Fz_fb = rank_Fzf(2);
    
    % find SA for lighter loaded front tire
    for SA = 0:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_fs,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_fs,SR,SA+0.1,IA,P,V);

        SA_optimized_fs = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_fs = SA;            
            break
        end
    end  
    
    % find SA for heavier loaded front tire
    for SA = SA_optimized_fs:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_fb,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_fb,SR,SA+0.1,IA,P,V);
        
        SA_optimized_fb = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_fb = SA;      
            break
        end
    end      
    
    % assign slip angle values to FL and FR
    if Fz_fl<Fz_fr
        SA_fl = SA_optimized_fs;
        SA_fr = SA_optimized_fb;
    else
        SA_fl = SA_optimized_fb;
        SA_fr = SA_optimized_fs;
    end


    % % REAR TIRES

    % define the smaller and bigger Fz for rear tires
    rank_Fzr = sort([Fz_rl,Fz_rr]);
    Fz_rs = rank_Fzr(1);
    Fz_rb = rank_Fzr(2);

    % find SA for lighter loaded front tire
    for SA = 0:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_rs,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_rs,SR,SA+0.1,IA,P,V);

        SA_optimized_rs = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_rs = SA;            
            break
        end
    end  
    
    % find SA for heavier loaded front tire
    for SA = SA_optimized_rs:0.1:15  
        [Lat0,Long0] = tires(tyre_model,Fz_rb,SR,SA,IA,P,V);
        [Lat1,Long1] = tires(tyre_model,Fz_rb,SR,SA+0.1,IA,P,V);
        
        SA_optimized_rb = 10;
        if abs(Lat1-Lat0) < 2
            SA_optimized_rb = SA;      
            break
        end
    end      
    
    % assign slip angle values to RL and RR
    if Fz_fl<Fz_fr
        SA_rl = SA_optimized_rs;
        SA_rr = SA_optimized_rb;
    else
        SA_rl = SA_optimized_rb;
        SA_rr = SA_optimized_rs;
    end



    % limit maximum slip angle with max steering angle
    if del_max-del_ack>0
        max_SA = del_max-del_ack;
    else
        max_SA = 0;
    end

    % limit front slip angle with steering angle
    if SA_fl > max_SA
        SA_fl = max_SA;
    end

    if SA_fr > max_SA
        SA_fr = max_SA;
    end

    % obtain tire forces with front and rear loads
    [Fy_fl,~] = tires(tyre_model,Fz_fl,SR,SA_fl,IA,P,V);
    [Fy_fr,~] = tires(tyre_model,Fz_fr,SR,SA_fr,IA,P,V);
    [Fy_rl,~] = tires(tyre_model,Fz_fl,SR,SA_rl,IA,P,V);
    [Fy_rr,~] = tires(tyre_model,Fz_fr,SR,SA_rr,IA,P,V);

    Fy_fl = Fy_fl*tc_lat*sen_lat;
    Fy_fr = Fy_fr*tc_lat*sen_lat;
    Fy_rl = Fy_rl*tc_lat*sen_lat;
    Fy_rr = Fy_rr*tc_lat*sen_lat;
   
    Lat = Fy_fl + Fy_fr + Fy_rl + Fy_rr;

    % required force for car to corner
    corner_f = (mass*V^2)/R; 

    % if required force to corner higher than max force tyre can provide
    if corner_f > Lat 
        vel = V-0.1; 
        
        break
    end
end

del = del_ack + min(SA_fr,SA_fl);


fprintf("speed is " +vel*3.6+' kmh \n')
fprintf("steering angle is " +del+' deg \n')