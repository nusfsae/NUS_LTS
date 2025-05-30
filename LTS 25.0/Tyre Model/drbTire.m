% Tire data visualization and analysis

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Tyre Model')
HoosierLC0 = mfeval.readTIR('Hoosier_6_18_10_LC0_C2000.TIR');
HoosierR20 = mfeval.readTIR('HoosierR20.TIR');
cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Vehicle Model')

% Known variables
mass = 274;
air_density = 1.196;
frontel_area = 1.157757;
CLc = 3.782684;
camber = 0;
phit = 0;
P = convpres(10, 'psi', 'Pa'); % convert psi to pa
useMode = 121;
tyre_model = HoosierLC0;
max_speed = 40;
max_steer = 0.565;
curv = 8.374;
tc_lat = 0.67;
sen_lat = 1;
tc_long = 0.6077;
sen_long = 1;
max_rpm = 5500;
FDR = 3.36;
R = 0.2032;
sen_lat = 1;


figure 

% zero long slip
for V = 10:10:40
    V = V/3.6;
    Reaction_f = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
    Flist = zeros(15/0.1,2);

    for alphadeg = 0:0.1:15

        alpharad = deg2rad(alphadeg);
        inputsMF = [Reaction_f 0 alpharad camber phit V P];
        [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
        Lat = abs(outMF(2));
        index = int32(((alphadeg-0)/0.1)+1);
        %disp(index)
        Flist(index,1) = Lat*tc_long;
        Flist(index,2) = alphadeg;


    end
    %f = fit(Flist(:,2),Flist(:,1),'poly2');
    plot(Flist(:,2),Flist(:,1),"--",'DisplayName',"Speed = "+ V*3.6+"km/h "+"[LC0]",'LineWidth',2);

    legend
    lg = legend
    lg.FontSize = 12
    hold on
end

sgtitle("Lateral Tire Data")
xlabel("Slip Angle (degree)",'FontSize',14)
ylabel("Lateral Force (N)",'FontSize',14)   

tyre_model = HoosierR20;

% zero long slip
for V = 10:10:40
    V = V/3.6;
    Reaction_f = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
    Flist = zeros(15/0.1,2);

    for alphadeg = 0:0.1:15

        alpharad = deg2rad(alphadeg);
        inputsMF = [Reaction_f 0 alpharad camber phit V P];
        [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
        Lat = abs(outMF(2));
        index = int32(((alphadeg-0)/0.1)+1);
        %disp(index)
        Flist(index,1) = Lat*tc_long;
        Flist(index,2) = alphadeg;


    end
    %f = fit(Flist(:,2),Flist(:,1),'poly2');
    plot(Flist(:,2),Flist(:,1),"-",'DisplayName',"Speed = "+ V*3.6+"km/h "+"[R20]",'LineWidth',2);

    legend
    lg = legend
    lg.FontSize = 12
    hold on
end

sgtitle("Lateral Tire Data")
xlabel("Slip Angle (degree)",'FontSize',14)
ylabel("Lateral Force (N)",'FontSize',14)   



% tyre_model = HoosierR20;
% 
% figure
% 
% % zero long slip
% for Fz = 500:500:3000
%     V = V/3.6;
%     % Fz = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
% 
%     Flist = zeros(15/0.1,2);
%     for alphadeg = 0:0.1:15        
% 
%         [Lat,Long0] = tires(tyre_model,Fz,0,alphadeg,IA,9,10);
% 
%         index = int32(((alphadeg-0)/0.1)+1);
%         %disp(index)
%         Flist(index,1) = Lat*tc_long;
%         Flist(index,2) = alphadeg;
% 
% 
%     end
%     %f = fit(Flist(:,2),Flist(:,1),'poly2');
%     plot(Flist(:,2),Flist(:,1),"-",'DisplayName',"Fz = "+ Fz+"N "+"[R20]");
% 
%     legend
%     lg = legend
%     lg.FontSize = 12
%     hold on
% end
% 
% 
% % justify static pressure
% 
% clear P

% figure 
% 
% 
% 
% 
% for alphadeg = 3:2:14
%     V = 10;
%     Reaction_f = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
%     Plist = zeros((13-7)/0.1+1,2);
%     index = 0;
%     for P = 7:0.1:13
%         Ppa = convpres(P, 'psi', 'Pa');
%         alpharad = deg2rad(alphadeg);
%         inputsMF = [Reaction_f 0 alpharad camber phit V Ppa];
%         [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
%         Lat = abs(outMF(2));
%         index = index +1;
%         %disp(index)
%         Plist(index,1) = Lat*tc_long;
%         Plist(index,2) = P;
% 
% 
%     end
%     %f = fit(Flist(:,2),Flist(:,1),'poly2');
%     plot(Plist(:,2),Plist(:,1),"-",'DisplayName',"Slip Angle = "+ alphadeg+" degree ");
% 
%     legend
%     lg = legend
%     lg.FontSize = 12
%     hold on
% end
% 
% 
% sgtitle("Pressure Sensitivity at 36 km/h")
% xlabel("Tire Pressure (psi)")
% ylabel("Lateral Force (N)")  
% xlim([7 13]);
% ylim([600 1200])
% 
% 
% % camber sensitivity at 10deg SA
% 
% clear P camber IAlist index Ppa
% 
% figure 
% 
% 
% index = 0;
% 
% 
% for P = 7:2:13
%     V = 45/3.6; % skipad speed 35kmh
%     Reaction_f = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
%     IAlist = zeros((5+5)/0.1,2);
%     index = 0;
%     for IA = -5:0.1:10
%         Ppa = convpres(P, 'psi', 'Pa');
%         alpharad = deg2rad(2);
%         camber = deg2rad(IA);
%         inputsMF = [Reaction_f 0 alpharad camber phit V Ppa];
%         [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
%         Lat = abs(outMF(2));
%         index = index +1;
%         %disp(index)
%         IAlist(index,1) = Lat*tc_long;
%         IAlist(index,2) = IA;
% 
% 
%     end
% 
%     plot(IAlist(:,2),IAlist(:,1),"-",'DisplayName',"Pressure = "+ P+" psi");
% 
%     legend
%     lg = legend
%     lg.FontSize = 12
%     hold on
% end
% 
% 
% sgtitle("Camber Sensitivity at 45 km/h (SA = 2 deg)")
% xlabel("Inclination Angle (deg)")
% ylabel("Lateral Force (N)")  
% xlim([-5 10]);
% ylim([200 800])
% 
% 
% 
% 
% % camber sensitivity at 9psi
% 
% clear P camber IAlist index Ppa
% 
% figure 
% 
% 
% index = 0;
% 
% 
% for SA = 2:4:14
%     V = 45/3.6; % skipad speed 35kmh
%     Reaction_f = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
%     IAlist = zeros((5+5)/0.1,2);
%     index = 0;
%     for IA = -5:0.1:10
%         Ppa = convpres(9, 'psi', 'Pa');
%         alpharad = deg2rad(SA);
%         camber = deg2rad(IA);
%         inputsMF = [Reaction_f 0 alpharad camber phit V Ppa];
%         [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
%         Lat = abs(outMF(2));
%         index = index +1;
%         %disp(index)
%         IAlist(index,1) = Lat*tc_long;
%         IAlist(index,2) = IA;
% 
% 
%     end
% 
%     plot(IAlist(:,2),IAlist(:,1),"-",'DisplayName',"Slip Angle = "+ SA+" deg");
% 
%     legend
%     lg = legend
%     lg.FontSize = 12    
%     hold on
% end
% 
% 
% sgtitle("Camber Sensitivity at 45 km/h (9 Psi)")
% xlabel("Inclination Angle (deg)")
% ylabel("Lateral Force (N)")  
% xlim([-5 8]);
% ylim([200 1200])
% 
% 
% 
% 
% 
% % justify static pressure
% 
% clear P
% 
% figure 
% 
% 
% 
% 
% alphadeg=10;
% V = 10;
% Reaction_f = (mass*9.81 + 0.5*CLc*frontel_area*V^2)/4;
% Plist = zeros((13-7)/0.1+1,2);
% index = 0;
% for P = 7:2:13
%     Ppa = convpres(P, 'psi', 'Pa');
%     alpharad = deg2rad(alphadeg);
%     inputsMF = [Reaction_f 0 alpharad camber phit V Ppa];
%     [ outMF ] = mfeval(tyre_model, inputsMF, useMode);
%     Lat = abs(outMF(2));
%     index = index +1;
%         %disp(index)
%     Plist(index,1) = Lat*tc_long;
%     Plist(index,2) = P;
% 
% 
% end
%     %f = fit(Flist(:,2),Flist(:,1),'poly2');
% plot(Plist(:,2),Plist(:,1),"x",'DisplayName',"Slip Angle = "+ alphadeg+" degree ");
% 
% legend
% 
% hold on
% 
% xlim([6 14])
% ylim([600 1000])