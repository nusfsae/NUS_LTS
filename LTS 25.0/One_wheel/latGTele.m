% Return array of Lateral G across whole track
function Lat_Accel = latGTele(lsp,C2)

%a = (v.^2-u.^2)/2*s
len = length(lsp);
Lat_Accel = zeros(len,1);

for point = 1:len
    v = lsp(point);  
    r = 1/C2(point);    
    a = ((v^2)/r)/9.81;
    Lat_Accel(point) = a;
end
end