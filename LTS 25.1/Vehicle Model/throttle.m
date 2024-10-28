function throttle_graph = throttle(lsp,dist)
Long_Accel = longG(lsp,dist);
len = length(lsp);

throttle_graph = zeros(len,1);

for i = 1:len
    actual = 0;

    if Long_Accel(i)>0
        actual = Long_Accel(i);    
    end

    
    full = 0.8;
    
    tt= (actual/full)*100;
    if tt>100
        tt =100;
    end
    throttle_graph(i) = tt; 
    

end
end