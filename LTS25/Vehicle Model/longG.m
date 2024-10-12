function Long_Accel = longG(lsp,dist)

%a = (v.^2-u.^2)/2*s
len = length(lsp);
Long_Accel = zeros(len,1);

for point = 1:len-1
    u = lsp(point);
    v = lsp(point+1);
    
    s = dist(point+1) - dist(point);
    
    a = ((v^2-u^2)/(2*s))/9.81;
    Long_Accel(point) = a;
end
end