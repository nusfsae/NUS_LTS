% Accel Track Generation

% Load data, convert to SI unit, and calculate time step

cd('C:\Users\PC5\Documents\Patrick\FSAE LTS\NUS_LTS-main\LTS 25.0\Track Model')


leng = 75;
discretion = 10;

size = leng*discretion;
pos = struct('x', zeros(size,1), 'y', zeros(size,1));
dist = zeros(size,1);
C2 = zeros(size,1);





% Curvature vs distance
for i = 1:size
    C2(i) = 0;
    dist(i) = (i-1)/discretion;
    pos.x(i) = dist(i);
    pos.y(i) = 0;
    
end


% Plotting the map
%scatter (pos.x, pos.y)
figure
hold on
%plot (dist,C1)
plot (dist,C2)
sgtitle('Track Model Curvature')
xlabel('Distance (m)')
ylabel('Curvature (1/m)')

% creating distance based mesh
meshsize = 1; % metres
s = floor(dist(end)/meshsize);
Dist = 0 : meshsize : s*meshsize;
c2 = interp1(dist,C2,Dist);
%figure
%plot(Dist,c2)

% creating distance based map
x = interp1(dist,pos.x,Dist);
y = interp1(dist,pos.y,Dist);
figure
scatter(x,y,'.')
axis equal
sgtitle('Track Model Position')
xlabel('X coordinates')
ylabel('Y coordinates')

pos.x = x/10;
pos.y = y;

pos.x(1) = 0;
pos.y(1) = 0;

C2 = transpose(C2);
dist = transpose(dist);

% CHANGE HERE ```
save("75m Accel.mat",'pos','C2','dist')
% CHANGE HERE ^^^

clear x y C1 C3 c2 Dist i meshsize s size t vel theta

