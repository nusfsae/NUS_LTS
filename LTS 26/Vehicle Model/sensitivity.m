%% sensitivity
% sensitivity settings
close all
var_min = 190;
var_max = 210;
var_step = 20;
sen_list = linspace(var_min,var_max,var_step);
sen.time = zeros(var_step,1);
sen.values = zeros(var_step, 1); 
if true
    for sen_num = 1:length(sen_list)
        load(endurance);
        num = length(C2);
        sim = struct();
        car = sen_list(sen_num);
        GGV;dynamics;plotter;
        close all
        sen.time(sen_num,1) = laptime;
        sen.values(sen_num,1) = sen_list(sen_num);
        clear sim
    end
end
%% plot result
figure
resultplot = plot(sen.values(:),sen.time(:));ylabel('Variable of interest');
title('Sensitivity Study');
grid on

resultplot.LineStyle = '-';       % Dashed line style
resultplot.LineWidth = 2;          % Thicker line
resultplot.Marker = 'x';           % Circle markers
resultplot.MarkerEdgeColor = 'r';  % Blue marker fill color