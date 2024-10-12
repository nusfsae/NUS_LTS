function brake_graph = brake(lsp,dist,mass,den,A,cl,camber,C2,tyre_model)

Long_Accel = longG(lsp,dist);
len = length(lsp);
slip_ang = slip_angle(C2);

brake_graph = zeros(len,1);

for i = 1:len
    actual = 0;
    v = lsp(i);
    SA = slip_ang(i);

    RF = mass*9.81 + 0.5*den*A*cl*v^2;
    [~,Long,~] = tyres(camber,SA,RF,tyre_model);

    full = Long/(mass*9.81);

    if Long_Accel(i)<0
        actual = abs(Long_Accel(i));    
    end   
   
    
    bb= (actual/full)*100;

    if bb>100
        bb =100;
    end

    brake_graph(i) = bb; 
    

end
end