function throttle_graph = throttle(lsp,dist,max_torque,FDR,R,mass)
Long_Accel = longG(lsp,dist);
len = length(lsp);

throttle_graph = zeros(len,1);

for i = 1:len
    actual = 0;

    if Long_Accel(i)>0
        actual = Long_Accel(i);    
    end
    % Powertrain maximum tractive force
    if lsp(i)<23.6298
        torque = max_torque;
    else
        torque = -5.4*lsp(i) + 297.181;
    end
    F_powertrain = torque*FDR/R;
    
    full = F_powertrain/mass/9.81;    
    tt= (actual/full)*100;
    if tt>100
        tt =100;
    end
    throttle_graph(i) = tt; 
end
end