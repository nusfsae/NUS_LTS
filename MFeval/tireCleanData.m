load('B1654run35.mat', 'FX', 'FY', 'FZ', 'IA', 'SA', 'SL', 'P')
dataClean = (IA <0.5) & (IA > -0.5) & (FZ < -550) & (FZ > -650) & (P > 80) & (P < 85);
FX_clean = FX(dataClean);
FY_clean = FY(dataClean);
SA_clean = SA(dataClean);
SL_clean = SL(dataClean);

%% bar chart of FZ, P, IA
figure
yyaxis left
plot(FZ)
hold on;
yyaxis right
plot(P)
hold on;
plot(IA)

xlim([0 30000])
legend show;
hold off;
