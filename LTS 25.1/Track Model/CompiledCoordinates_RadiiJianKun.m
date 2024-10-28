% Specify the path to the Excel file
excelFile = 'C:\Users\Aero1\Desktop\23 EV track.csv';
% solidworksfile = 'C:\Users\Aero1\Desktop\Part1.DXF';
% Read the data from the Excel file
data = xlsread(excelFile);
AT = zeros(size(data));

for i = 1:size(data,1) %reading data to create first unordered list
    Order(i,1) = data(i,1)/1.5;%
    Order(i,2) = data(i,2)/1.5;%3.5 for ice 1.5 ev 1.25 sg gp

end
% 3.577142857142857e+02 -2.414285714285714e+02
%Take starting point in a straight line. the Coordinates will be ( 3.577142857142857e+02,-2.414285714285714e+02)
%Order = zeros(size(data)); %this is the ordered list. add first coordinate in
p1 = Order(1,1)/1.5;%1252
p2 = Order(1,2)/1.5;%-845
%Order(1,1) = p1    ;%3.577142857142857e+02
%Order(1,2) = p2   ;%-2.414285714285714e+02
%pointToremove = [p1,p2];%3.577142857142857e+02 2.414285714285714e+02

%index = find(AT(:,1) == pointToremove(1) & AT(:,2) == pointToremove(2));
%index = find(ismember(AT,pointToremove, 'rows')); %find index of the first point

%AT(index, :) = []; %remove the first point from the unordered list
    
%Given point Coordinates
% for i = 1: size(data,1)-1
%     
%     % List of point Coordinates
%     
%     % Calculate the Euclidean distances between the given point and each
%     distances = sqrt((AT(:, 1) - Order(i,1)).^2 + (AT(:, 2) - Order(i,2)).^2);
% 
%     % Find the index of the closest point
%     [~, closestIndex] = min(distances);
% 
%     % Get the closest point Orders
%     closestPoint = AT(closestIndex, :);
%     AT(closestIndex, :) = []; %remove closest point after finding it
%     % add closest point to the order list
%     Order(i+1,1) = closestPoint(1,1);
%     Order(i+1,2) = closestPoint(1,2);
    

%end
% Order(1,1) = Order(1,1)/3.5;
% Order(1,2) = Order(1,2)/3.5;
%creating the track on the figure
radiiList = zeros(size(Order(:,1)));
CentreCoord = zeros(size(Order));
figure;
plot(Order(:,1)-p1, Order(:,2)-p2, 'b+', 'MarkerSize', 5);%- 3.577142857142857e+02 +2.414285714285714e+02  -91.1568571 +36.8431429
hold on;
xlabel('X Order');
ylabel('Y Order');
title('Generated Track Coordinates with Calculated Radii');
grid on;
axis equal;
%Display the generated Orders
%disp('Generated Orders:');
dist = zeros(1,length(Order));

for i = 1:length(Order)
     if i == 1 % two conditions that need if statements which is first and last point
        x1 = Order(length(Order),1); % arranging the 1st 2nd and 3rd points and inputting into function to get radii and sol
        y1 = Order(length(Order),2);
        x2 = Order(i,1);
        y2 = Order(i,2);
        x3 = Order(i+1,1);
        y3 = Order(i+1,2);
        [radii,sol] = InstantaneousRadiiFxn_(x1,y1,x2,y2,x3,y3); 
        dist(i)=0.0001;
     elseif i == length(Order)
        x1 = Order(i-1,1);
        y1 = Order(i-1,2);
        x2 = Order(i,1);
        y2 = Order(i,2);
        x3 = Order(1,1);
        y3 = Order(1,2);
        [radii,sol] = InstantaneousRadiiFxn_(x1,y1,x2,y2,x3,y3);
        dist(i)=0.0002 +dist(i-1);
     else
        x1 = Order(i-1,1);
        y1 = Order(i-1,2);
        x2 = Order(i,1);
        y2 = Order(i,2);
        x3 = Order(i+1,1);
        y3 = Order(i+1,2);
        [radii,sol] = InstantaneousRadiiFxn_(x1,y1,x2,y2,x3,y3);
        dist(i)= distt(x1,y1,x2,y2)+ i/10000 +dist(i-1); % 
    end
    hold on
    %radiiList1(i,1) = radii;
    radiiList(i,1) = (1/radii+i/100000); % list of the instantaneous radii at every point. however there are some which Nan and some which are large which i am not sure how to categorise to a straight
    radiiList(isnan(radiiList))=0.05+i/10000; %just changed from 0.005  15 june
    radiiList(radiiList==0)=0.05+i/10000;
    CentreCoord(i,1) = sol(1,1);
    CentreCoord(i,2) = sol(2,1);
    %if you want to see the centre coord for all points 
    %plot(sol(1),sol(2),'g*')
    % if you want to see the circles formed by the 3 coordinatees
    %viscircles([sol(1) sol(2)],radii,"LineWidth",0.5);
    
    hold off;  
end

C2=radiiList';
Or=Order';

% Create struct
pos.x = Or(1,:)-p1 ;%- 3.577142857142857e+02
pos.y = Or(2,:)-p2;%+2.414285714285714e+02


% dist=dist/3.181818;


%save("23 EV endurance track.mat","pos",'C2','dist')
function[radii,sol] = InstantaneousRadiiFxn_(x1,y1,x2,y2,x3,y3)

gradi1 = (y2-y1)/(x2-x1); % gradient of line between first two Orders
B = -1/gradi1; % gradient of line normal to Line 1 which is Line 3
G = (y1+y2)/2; % midpoint which is a point that lies in Line 3
D = (x1+x2)/2; % midpoint x which is a point that lies in Line 3
C = G - D*B;
%%%%%%%
cc=y2-gradi1*x2;
newy=gradi1*x3+cc;
%REPEAT FOR L4. HARDCODED
gradi1 = (y2-y3)/(x2-x3); % gradient of line between first two Orders
B2 = -1/gradi1; % gradient of line normal to Line 1 which is Line 3
G2 = (y3+y2)/2; % midpoint which is a point that lies in Line 3
D2 = (x3+x2)/2; % midpoint x which is a point that lies in Line 3
C2 = G2 - D2*B2;

Leqn = [-B,1 ; -B2,1];
offset = [C ; C2];
sol = linsolve(Leqn,offset);

radii = sqrt((sol(2)-y2).^2 + (sol(1)-x2).^2);
if newy>y3 && x2>x1
    radii=-radii;

elseif newy<y3 && x2<x1
    radii=-radii;
end
end

function [dist]=distt(x11,y11,x22,y22)

dist=sqrt((x22 - x11)^2 + (y22 - y11)^2);

end
