%% plotter

% calculate yaw rate
sim.yaw = sim.ay./sim.speed;
% calculate lap time
dt = 1./max(sim.speed,0.001);
if sim.speed(1) == 0
    dt(1) = 0;
end
t = cumtrapz(dist,dt);
laptime = t(end);
% calculate steering angle
sim.delta = rad2deg(findDelta(abs(sim.ay),sim.speed));
yneg = find(sim.ay<0);
sim.delta(yneg) = -1*sim.delta(yneg);
% slip ratio
sim.Sxfl = findSxfl(sim.ax,sim.speed);
sim.Sxfr = findSxfr(sim.ax,sim.speed);
sim.Sxrl = findSxrl(sim.ax,sim.speed);
sim.Sxrr = findSxrr(sim.ax,sim.speed);
% slip angle
sim.Safl = rad2deg(findSafl(abs(sim.ay),sim.speed));
sim.Safr = rad2deg(findSafr(abs(sim.ay),sim.speed));
sim.Sarl = rad2deg(findSarl(abs(sim.ay),sim.speed));
sim.Sarr = rad2deg(findSarr(abs(sim.ay),sim.speed));
sim.Safl(yneg) = -1*sim.Safl(yneg);
sim.Safr(yneg) = -1*sim.Safr(yneg);
sim.Sarl(yneg) = -1*sim.Sarl(yneg);
sim.Sarr(yneg) = -1*sim.Sarr(yneg);

% car statistics plots
figure
tiledlayout(6,1);
% plot speed profile
nexttile;
plot(dist,sim.speed*3.6);ylabel('speed (km/h)');ylim([0 140]);
title('Vehicle Speed')
% plot ay
nexttile
plot(dist,sim.ay/9.81);ylabel('ay (G)');ylim([-3 3]);
title('Lateral Acceleration')
% plot ax
nexttile
plot(dist,sim.ax/9.81);ylabel('ax (G)');ylim([-3 3]);
title('Longitudinal Acceleration')
% plot yaw rate
nexttile
plot(dist,rad2deg(sim.yaw));ylabel('dpsi (deg/s)');ylim([-180 180]);
title('Yaw Rate')
% plot steering
nexttile
plot(dist,sim.delta);ylabel('Steering Angle (deg)');ylim([-rad2deg(del_max) rad2deg(del_max)]);
title('Steering Angle')
% plot time
nexttile
plot(dist,t);ylabel('time (s)');ylim([0 t(end)]);
title('Time')
xlabel('distance(m)')

% tire data plots
% slip ratio
figure
tiledlayout(4,1);
nexttile;
plot(dist,sim.Sxfl);ylabel('Slip Ratio');ylim([-0.1 0]);
title('FL')
nexttile;
plot(dist,sim.Sxfr);ylabel('Slip Ratio');ylim([-0.1 0]);
title('FR')
nexttile;
plot(dist,sim.Sxrl);ylabel('Slip Ratio');ylim([-0.1 0.1]);
title('RL')
nexttile;
plot(dist,sim.Sxrr);ylabel('Slip Ratio');ylim([-0.1 0.1]);
title('RR')
% slip angle
figure
tiledlayout(4,1);
nexttile;
plot(dist,sim.Safl);ylabel('Slip Angle');ylim([-10 10]);
title('FL')
nexttile;
plot(dist,sim.Safr);ylabel('Slip Angle');ylim([-10 10]);
title('FR')
nexttile;
plot(dist,sim.Sarl);ylabel('Slip Angle');ylim([-10 10]);
title('RL')
nexttile;
plot(dist,sim.Sarr);ylabel('Slip Angle');ylim([-10 10]);
title('RR')



%%

% plot color track map speed data
figure
scatter(pos.x,pos.y,10,sim.speed*3.6,'filled','o');
colormap(jet);
cb = colorbar;
ylabel(cb, 'Speed (km/h)');
title('Race Track with Speed Visualization', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);
grid on;
axis equal;