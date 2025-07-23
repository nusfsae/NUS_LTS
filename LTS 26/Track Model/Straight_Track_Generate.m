cd('D:\Patrick\VD SIM\Point Mass\Track Model')

length = 75; %meters
frequency = 100;

size = length*frequency;

C2 = zeros(size,1);
dist = zeros(size,1);

for point = 1:size
    dist(point) = point/100;
end

save("75m Accel.mat",'C2','dist')