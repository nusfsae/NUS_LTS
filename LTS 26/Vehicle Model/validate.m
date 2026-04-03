%% Simulink vs MoTeC validation - SPEED + LONG G
out = sim("accelSim");

%% Extract Simulink data
d_sim    = out.distance.Data;     % m
v_sim    = out.speed.Data;        % km/h
longG_sim = out.longG.Data;       % g

%% Load MoTeC data
motec        = load('25 accel.mat');
speed_motec  = motec.Corr_Speed.Value;       % km/h
dist_motec   = motec.Corr_Dist.Value;        % m
longG_motec  = motec.Smooth_Long_G.Value;    % g

%% Normalize MoTeC start point
dist_motec = dist_motec - dist_motec(1);

%% Remove repeated MoTeC distance points
[dist_motec_u, ia] = unique(dist_motec, 'stable');
speed_motec_u  = speed_motec(ia);
longG_motec_u  = longG_motec(ia);

%% Make Simulink distance unique
[d_sim_u, ib] = unique(d_sim, 'stable');
v_sim_u      = v_sim(ib);
longG_sim_u  = longG_sim(ib);

%% Interpolate onto common distance grid
d_common = linspace(0, 75, 300);

v_sim_i       = interp1(d_sim_u,      v_sim_u,       d_common, 'linear', 'extrap');
v_motec_i     = interp1(dist_motec_u, speed_motec_u,  d_common, 'linear', 'extrap');
longG_sim_i   = interp1(d_sim_u,      longG_sim_u,   d_common, 'linear', 'extrap');
longG_motec_i = interp1(dist_motec_u, longG_motec_u, d_common, 'linear', 'extrap');

%% ── PLOT 1: Speed Comparison ─────────────────────────────────────────────
figure
set(gcf, 'Color', 'k')
plot(d_common, v_sim_i,   'w-', 'LineWidth', 2); hold on
plot(d_common, v_motec_i, 'r-', 'LineWidth', 2);
xlabel('Distance (m)', 'Color', 'w')
ylabel('Speed (km/h)',  'Color', 'w')
title('Simulink vs MoTeC — Speed', 'Color', 'w')
xlim([0 75]); ylim([0 130])
ax = gca; ax.Color='k'; ax.XColor='w'; ax.YColor='w';
ax.GridColor=[1 1 1]; ax.GridAlpha=0.2; grid on
legend({'Speed - Simulink','Speed - MoTeC'}, 'TextColor','w','Location','best')

%% ── PLOT 2: Speed Error ──────────────────────────────────────────────────
dv = v_sim_i - v_motec_i;
figure
set(gcf, 'Color', 'k')
plot(d_common, dv, 'y', 'LineWidth', 2); hold on
yline(0, 'w--', 'LineWidth', 1)
xlabel('Distance (m)',       'Color', 'w')
ylabel('Speed Error (km/h)', 'Color', 'w')
title('Speed Error: Simulink − MoTeC', 'Color', 'w')
xlim([0 75])
ax = gca; ax.Color='k'; ax.XColor='w'; ax.YColor='w';
ax.GridColor=[1 1 1]; ax.GridAlpha=0.2; grid on

%% ── PLOT 3: Long G Comparison ────────────────────────────────────────────
figure
set(gcf, 'Color', 'k')
plot(d_common, longG_sim_i,   'w-', 'LineWidth', 2); hold on
plot(d_common, longG_motec_i, 'r-', 'LineWidth', 2);
xlabel('Distance (m)',        'Color', 'w')
ylabel('Longitudinal G (g)',  'Color', 'w')
title('Simulink vs MoTeC — Long G', 'Color', 'w')
xlim([0 75])
ax = gca; ax.Color='k'; ax.XColor='w'; ax.YColor='w';
ax.GridColor=[1 1 1]; ax.GridAlpha=0.2; grid on
legend({'Long G - Simulink','Long G - MoTeC'}, 'TextColor','w','Location','best')

%% ── PLOT 4: Long G Error ─────────────────────────────────────────────────
dG = longG_sim_i - longG_motec_i;
figure
set(gcf, 'Color', 'k')
plot(d_common, dG, 'c', 'LineWidth', 2); hold on
yline(0, 'w--', 'LineWidth', 1)
xlabel('Distance (m)',      'Color', 'w')
ylabel('Long G Error (g)',  'Color', 'w')
title('Long G Error: Simulink − MoTeC', 'Color', 'w')
xlim([0 75])
ax = gca; ax.Color='k'; ax.XColor='w'; ax.YColor='w';
ax.GridColor=[1 1 1]; ax.GridAlpha=0.2; grid on

%% ── Summary Stats ────────────────────────────────────────────────────────
fprintf('\n=== Validation Summary ===\n')
fprintf('Speed  — RMSE: %.3f km/h  |  Max |err|: %.3f km/h\n', ...
        rms(dv), max(abs(dv)))
fprintf('Long G — RMSE: %.4f g     |  Max |err|: %.4f g\n',    ...
        rms(dG), max(abs(dG)))