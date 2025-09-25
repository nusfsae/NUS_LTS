%% test


%% tire test

% car parameters
para = H1675;
mass = 199.7+66;
SR = 0;
den =1;
V = 10;
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

%% combine slip
clear fy fx;
sa_range = deg2rad(linspace(-15, 15, 100));
sx_range = linspace(-0.15, 0.15, 100);
fz_range = linspace(400, 1500, 4);

[X, Y] = meshgrid(sa_range, sx_range);
sa_range = X(:);
sx_range = Y(:);

for i = 1:numel(sx_range)
    [FY, FX] = MF52(sx_range(i), sa_range(i), fz_range(2), 0, para);
    fy(i) = FY;
    fx(i) = FX;
end

figure(3); clf; t = tiledlayout(2,3);
title(t, 'Combined Slip')
nexttile
scatter(fy, fx)
xlabel('Fy [N]')
ylabel('Fx [N]')
axis equal
grid on

nexttile
scatter(rad2deg(sa_range), fy, [], sx_range, '.')
colorbar
ylabel('Fy [N]')
xlabel('Slip Angle [deg]')

nexttile
scatter(sx_range, fx, [], sa_range, '.')
colorbar
ylabel('Fx [N]')
xlabel('Slip Ratio [-]')

nexttile
title('Sx-Fy Dependency')
scatter(sx_range, fy, [], sa_range, '.')
colorbar
ylabel('Fy [N]')
xlabel('Slip Ratio [-]')

nexttile
title('Sa-Fx Dependency')
scatter(rad2deg(sa_range), fx, [], sx_range, '.')
colorbar
ylabel('Fx [N]')
xlabel('Slip Angle [deg]')
ylim([-3000 3000])



%% find correction factor
t = 4.963;   % skidpad timing
radius = 7.625+1.5;
v = 2*pi*radius/t;
lat = v^2/radius;
mass = 199.7+66;
% force require to maintain skidpad
Fsum = lat*mass;
% read tire force from tire graph
cf = Fsum/(4*1800)
