% Specify the path to the Excel file
excelFile = 'C:\Users\Aero1\Desktop\CoordinatesCleanedver4.csv';

% Read the data from the Excel file
data = xlsread(excelFile);
AT = zeros(size(data));

for i = 1:size(data,1) %reading data to create first unordered list
    AT(i,1) = data(i,1);%/3.5
    AT(i,2) = data(i,2);%/3.5

end
% 3.577142857142857e+02 -2.414285714285714e+02
%Take starting point in a straight line. the Coordinates will be ( 3.577142857142857e+02,-2.414285714285714e+02)
    Order = zeros(size(data)); %this is the ordered list. add first coordinate in
    Order(1,1) =  1252  ;%3.577142857142857e+02
    Order(1,2) =   -845 ;%-2.414285714285714e+02
    pointToremove = [ 1252,-845];
    
    index = find(AT(:,1) == pointToremove(1) & AT(:,2) == pointToremove(2));
    %index = find(ismember(AT,pointToremove, 'rows')); %find index of the first point
    
    AT(index, :) = []; %remove the first point from the unordered list