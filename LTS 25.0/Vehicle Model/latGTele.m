% Return array of Lateral G across whole track
function sim = latGTele(sim,C2)

%a = (v.^2-u.^2)/2*s
len = length(sim.speed);

for point = 1:len
    v = sim.speed(point);  
    r = 1/C2(point);    
    a = -((v^2)/r)/9.81;
    sim.latG(point) = a;
end
end