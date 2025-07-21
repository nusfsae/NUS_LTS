% Return Longitudinal G across the track
function sim = longGTele(sim,dist)

lsp = sim.speed;
%a = (v.^2-u.^2)/2*s
len = length(lsp);
for point = 1:len-1
    u = lsp(point);
    v = lsp(point+1);
    
    s = dist(point+1) - dist(point);
    
    a = ((v^2-u^2)/(2*s))/9.81;
    sim.longG(point) = a;
end
end