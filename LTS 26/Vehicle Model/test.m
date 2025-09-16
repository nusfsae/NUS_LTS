%% test


%% tire test

% car parameters
para = H1675;
mass = 199.7+66+500;
SR = 0;
den =1;
V = 30;
CLs = 4;
farea = 1;
Fz = mass*9.81/4 + 0.5*den*(V^2)*CLs*farea/4;
IA = 0;

% Fy vs SA (1-15deg)
Fygraph = zeros(15,2);
for i = 1:15
    SA = deg2rad(i-1);
    [Fy,Fx] = MF52(SR,SA,Fz,IA,para);
    Fygraph(i,1) = i;
    Fygraph(i,2) = -Fy;
end
figure
nexttile
plot(Fygraph(:,1),Fygraph(:,2));
xlabel('Slip Angle');
ylabel('Fy (N)');
title('Fy vs SA');

% Fx vs SR (0-0.2)
SA = 0;
Fxgraph = zeros(20,2);
for i = 1:20
    SR = (i-1)/100;
    [Fy,Fx] = MF52(SR,SA,Fz,IA,para);
    Fxgraph(i,1) = SR;
    Fxgraph(i,2) = Fx;
end
nexttile
plot(Fxgraph(:,1),Fxgraph(:,2));
xlabel('Slip Ratio (SR)');
ylabel('Fx (N)');
title('Fx vs SR');


%% find correction factor
t = 4.963;   % skidpad timing
radius = 7.625+1.5;
v = 2*pi*radius/t;
lat = v^2/radius;
mass = 199.7+66;
Fsum = lat*mass;
cf = Fsum/(4*1800)