%this function finds the local minima of a nx2 array
%this code is not efficient at all
function min_list = localfinder(array)

len = length(array);
count = 0; %number of local minima

for dot = 2:len-1
    if  array(dot,1)<=array(dot+1,1) && array(dot,1)<array(dot-1,1)
        count = count+1; %this loop counts the number of local minima
    end
        
end

local_min = []; %create empty array for local minima
index_list = [];

for point = 2:len-1   
    if array(point,1)<=array(point+1,1) && array(point,1)<array(point-1,1)
        local_min(end+1) = array(point); 
        index_list(end+1) = point;
    end
       %fill in values of local minima and their indices
end

min_list = zeros(count,2);

for value = 1:count
    min_list(value,1) = local_min(value);
    min_list(value,2) = index_list(value);
end

end
