%Full code for the track modelling 
%Start with Splitting given image.jpg to coordinates
filename='C:\Users\Aero1\Desktop\Track Images\UStestrunEdit2.jpg'; %open file of the track image
E = imread(filename); %imread reads the image by its RGB value pixel by pixel
[row,col , ~] = size(E);

% num_img_h = 4   ; %define number col
% num_img_v = 2   ; %number of rows
% row_breaks = round(linspace(1,row,num_img_v+1),0);
% col_breaks = round(linspace(1,col,num_img_h+1),0);
% loop = 1;
% for iter1 = 1:num_img_v
%     iter1;
%     for iter2 = 1:num_img_h
%         iter2;
%         imwrite(E(row_breaks(iter1):row_breaks(iter1+1),col_breaks(iter2):col_breaks(iter2+1),1:3),['elizabeth', num2str(loop), '.tiff']);
%         loop = loop + 1;
%     end
% end
for i=1 : 686 % according to image size we go through each pixel which has a particular RGB value assigned to it
   for j =1: 1348
       if E(i,j,1) >= 110 && E(i,j,1) <= 240  %>=44  && E(i,j,1) <=50  this is where we choose our intended RGB value to convert to one singular arbitrary value
           E(i,j,1)=5; 
       %elseif E(i,j,1) == 255
           E(i,j,1)=5;
       %elseif E(i,j,1)>= 140 && E(i,j,1)<= 150
       %    E(i,j,1)=5;
       else % the other values are also assigned to another arbitrary value
           E(i,j,1)=3;  
       end
   end
end

filename = 'track1.csv';  % Name of the CSV file it is saved into 

writematrix(E(:,:,1), filename);


% Specify the path to the Excel file
excelFile = 'C:\Users\Aero1\Desktop\track1.csv';

% Read the data from the Excel file
[numData, ~] = xlsread(excelFile);

% Find the indices of cells with number 5
[rowIndices, colIndices] = find(numData == 5);

% Extract the coordinates from the indices
pointCoordinates = [colIndices, -rowIndices];

% Load the track data (assuming it is already plotted in MATLAB)
% Make sure the track data is stored in variables 'x' and 'y' representing the x and y coordinates respectively
% Adjust the track data loading code according to your specific data format
%excelFile = 'C:\Users\Aero1\Desktop\FSAE SIM SYAHIR\TrackCoordinates.csv';
data = xlsread(excelFile);
x = pointCoordinates(:,1);
y = pointCoordinates(:,2);
% Set the initial point
initialPoint = [382, -427]; %135,-68 For autocross  %changes for different tracks as well

% Set the radius for averaging nearby points
radius = 11;%15 for 23ice 8 FR EV
% different tracks might require different radius according to the picture

% Initialize arrays to store the predicted path and averaged points
predictedX = initialPoint(1);
predictedY = initialPoint(2);
averagedPointsX = [];
averagedPointsY = [];
nearbyPoints = [];
distance = sqrt((x(:,1) - predictedX).^2 + (y(:,1) - predictedY).^2);
for j = 1: size(distance,1)
    if distance(j) <= radius 
        nearbyPoints = [nearbyPoints;x(j),y(j)];
        %nearbyPoints = distance <= radius;
    end
end        
%Calculate the average of nearby points
avgX = mean([nearbyPoints(:,1);predictedX]);
avgY = mean([nearbyPoints(:,2);predictedY]);

%Store the averaged points in separate arrays
averagedPointsX = [averagedPointsX; avgX];
averagedPointsY = [averagedPointsY; avgY];
p = polyfit(nearbyPoints(:,1), nearbyPoints(:,2), 1);
nextX = avgX -1;  % Adjust the step size (1 in this example) based on your needs
nextY = polyval(p, nextX);  
predictedXL = [predictedX, nextX];
predictedYL = [predictedY, nextY];
predictedX = nextX;
predictedY = nextY;
nearbyPoints = [];
mList = [];
cList = [];
m2List = []; 
%Ptslist167 = [];
% Loop through each point in the trac
% Find the points within the radius of the current point
for i = 2:550 %%% This has to changed according to how big the track is 
    distance = sqrt((x(:,1) - predictedX).^2 + (y(:,1) - predictedY).^2);
    for j = 1: size(distance,1)
        if distance(j) <= radius
            nearbyPoints =[nearbyPoints;x(j),y(j)];
            %if i == 8 
            %    Ptslist167 = [Ptslist167;x(j),y(j)];   
            %end
            %nearbyPoints = distance <= radius;
        
        end
    end
    %Calculate the average of nearby points
    avgX = mean([nearbyPoints(:,1);predictedX]);
    avgY = mean([nearbyPoints(:,2);predictedY]);
    finishdist = sqrt((averagedPointsX(1)-avgX).^2+(averagedPointsY(1)-avgY).^2);
    if finishdist <= 5  && i >= 50 % rule to break the code when number of iterations is too many to avoid another iteration of track modelling
        break
    end
    %Store the averaged points in separate arrays
    averagedPointsX = [averagedPointsX; avgX];
    averagedPointsY = [averagedPointsY; avgY];

    %Use linear regression to predict the path of the current point
    %Fit a line to the nearby points including the averaged point using polyfit
    %Adjust the polynomial degree (1 for linear regression) based on your needs
    p = polyfit(nearbyPoints(:,1), nearbyPoints(:,2), 1);
    mList = [mList;p(1)];
    cList = [cList;p(2)];
    m = p(1);
    c = p(2);
    m2 = -1/m;
    m2List = [m2List;m2];
    c2 = avgY - m2*avgX;
    Leqn = [-m,1; -m2,1];
    offset = [c;c2];
    sol = linsolve(Leqn,offset);
    refdist =6;%9 for 23 ice 4 EV
    %refdist refers to the distance the next coordinates are guessed from.
    %again different track images require diff refdist. 
    % Calculate the change in x and y coordinates
    dx = sqrt(refdist^2 / (1 + m^2));
    dy = m * dx;
    x2 = sol(1) + dx;
    y2 = sol(2) + dy;   % Point on the line, 5 units away from the given point
    x3 = sol(1) - dx;
    y3 = sol(2) - dy;   % Point on the line, 5 units away from the given point
    d1 = sqrt((averagedPointsX(i-1)-x2).^2+(averagedPointsY(i-1)-y2).^2);
    d2 = sqrt((averagedPointsX(i-1)-x3).^2+(averagedPointsY(i-1)-y3).^2);
    if d1 > d2
        nextX = x2;
        nextY = y2;
    else
        nextX = x3;
        nextY = y3;
    end
    
    %Predict the next point based on the line equation
    %nextX = avgX + 5;  % Adjust the step size (1 in this example) based on your needs
    %nextY = polyval(p, nextX);

    %Store the predicted point in the arrays
    predictedXL = [predictedX; nextX];
    predictedYL = [predictedY; nextY];
    predictedX = nextX;
    predictedY = nextY;
    nearbyPoints = [];
end
% Plot the original track, averaged points, and the predicted path
figure;
% plot(x, y, 'b-','MarkerSize', 2);  % Original track in blue
hold on;
plot(averagedPointsX, averagedPointsY, 'b+', 'MarkerSize', 5);  % Averaged points in green circles
%plot(predictedX, predictedY, 'r--', 'LineWidth', 2);  % Predicted path in red dashed line
%plot(Ptslist167(:,1),Ptslist167(:,2),'o','MarkerSize',6);
legend('Averaged Points', 'Predicted Path');%'Original Track'
xlabel('X');
ylabel('Y');
title('Track Plot with Averaged Points and Predicted Path');
    grid on;
%% Only run the next section of the code after confirming a suitable track to model on based on radius and refdist

% Specify the path to the Excel file
%excelFile = 'C:\Users\Aero1\Desktop\23 EV track.csv';

% Read the data from the Excel file
%data = xlsread(excelFile);
%AT = zeros(size(data));

for i = 1:size(averagedPointsX,1)%*2 %reading data to create first unordered list
    if i >= size(averagedPointsX,1)+1 % for running start the i value has to change according to the number of points on track
        Order(i,1) = averagedPointsX(i-size(averagedPointsX,1))/4.9852838;
        Order(i,2) = averagedPointsY(i-size(averagedPointsX,1))/4.9852838;
    else
        Order(i,1) = averagedPointsX(i)/4.9852838;%*1.72133934;%
        Order(i,2) = averagedPointsY(i)/4.9852838;%*1.72133934;%3.5 for ice 1.5 ev 1.25 sg gp

    end
end

% 3.577142857142857e+02 -2.414285714285714e+02
%Take starting point in a straight line. the Coordinates will be ( 3.577142857142857e+02,-2.414285714285714e+02)
%Order = zeros(size(data)); %this is the ordered list. add first coordinate in
p1 = Order(1,1);%*1.72133934;%1252
p2 = Order(1,2);%*1.72133934;%-845
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
%CentreCoord = zeros(size(Order));
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
 
%for i = 274:546 %for more than 1 lap simulation
    %C2(1,i) = C2(1,i-273);
    %dist(1,i) = dist(1,i-273);
%end

% dist=dist/3.181818;


save("23 US test run track acw run.mat","pos",'C2','dist')
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


