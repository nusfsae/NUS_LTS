%calculate max accel performance with consider of tyre slip ratio
%R tyre radius

%open track model file
cd('D:\Patrick\VD SIM\LTS25\Track Model')

% select track
track = '75m Accel';
load(track)

cd ('D:\Patrick\VD SIM\LTS25\Tyre Model')
tyre = 'R25B_V2';
load(tyre)

% parameter add

R = 0.2032;%wheel radius
mass = 276;
air_density = 1.293;
frontel_area = 1.119492;
coef_drag = 1.619549;
coef_lift = 4.224138;%positive means downforce here
tyre_model = fit10psi; %select tyre with desired pressure
camber = 0;
FDR = 3.36;


[accel_profile,w_profile,throttle,a_profile,sr,lol,ff] = acc(R,dist,mass,0,0,tyre_model,air_density,frontel_area,coef_lift,coef_drag,FDR);
figure
plot(dist,a_profile);
figure
plot(dist,accel_profile);

cd ('D:\Patrick\VD SIM\LTS25\Vehicle Model')
lap_time_sim = Lap_Time_Simulation(accel_profile,dist);
disp(lap_time_sim);

function [accel_profile,w_profile,ttt,a_profile,sr,lol,ff] = acc(R,dist,mass,camber,alpha,model,den,A,CL,CD,FDR)

len = length(dist);
accel_profile = zeros(1,len);
w_profile = zeros(1,len);
ttt = zeros(1,len);
a_profile = zeros(1,len);
sr = zeros(1,len);
lol = zeros(1,len);
ff = zeros(1,len);

for point = 1:len-1
    vel = accel_profile(point);
    s = dist(point+1) - dist(point);
    FL = mass*9.81 + 0.5*den*A*CL*(vel^2); %normal force to tyres
    Drag = 0.5*den*A*CD*(vel^2);
    
    v_wheel = vel*(1+0.1);%SRT=0.2 convert to rpm
    
    if v_wheel>31.1412 %max rpm from motor
        v_wheel = 31.1412;
    end

    w_profile(point) = v_wheel;

    T = powertrain(v_wheel);
    F = FDR*T/R;  % max tractive force 
    ff(point) = F;

    [~,Long,~]= tyres(camber,alpha,FL/4,model); 
    
    Long = 4*Long*0.66; %tyre correlation factor
    lol(point) = Long;

    FT = 0.95*Long; % max tyre force pending F-SR relation 
    %FTmax = 165/R; %pending torque input  
    

    F_actual = min(F,FT);
    normalized = F_actual/Long;
    slipratio = normalized/5;
    sr(point) = slipratio;

    tt = 100*F_actual/F;
    
    ttt(point) = tt;

    a = (F_actual-Drag)/mass;
    
    v2 = sqrt(vel^2+ 2*a*s);
    w2 = (v2*(1+slipratio)); %cannot then change slipratio to 0.1

    if w2>31.1412
        v2 = 31.141/(1+slipratio);
    end

    a_profile(point) = ((v2^2-vel^2)/(2*s))/9.81;

    accel_profile(point+1) = v2;

end
end

function [Lat,Long,GH] = tyres(camber,alpha,Reaction_f,tyre_model)


Reaction_f = Reaction_f/2;
CS = tyre_model.CS(camber,Reaction_f);
S_H = tyre_model.S_H(camber,Reaction_f);
S_V = tyre_model.S_V(camber,Reaction_f);
mu = tyre_model.mu(camber,Reaction_f);
B = tyre_model.B(camber,Reaction_f);
E =  tyre_model.E(camber,Reaction_f);

a = (CS/Reaction_f)./mu.*(tan((alpha-S_H)*pi/180));
F_bar = sin((1/B).*atan(B.*(1-E).*a+E.*atan(B.*a)));
F_shift = F_bar*mu*Reaction_f;
Lat = (F_shift+S_V);
Long = (sqrt((mu*Reaction_f).^2-Lat.^2))*2;
Lat = Lat.*2;
GH = CS;
end


function torque = powertrain(wheelspeed)

if wheelspeed<23.6298
    torque = 169.58;

else
    torque = -5.4*wheelspeed + 297.181;
end

end
