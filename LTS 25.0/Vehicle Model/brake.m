function brake_graph = brake(lsp,dist,mass,den,A,CLs,CLc,CDs,CDc,camber,C2,tyre_model)

Long_Accel = longG(lsp,dist);
len = length(lsp);
slip_ang = slip_angle(C2);

brake_graph = zeros(len,1);

for i = 1:len
    curv = abs(1/C2(i));

    if curv > 30
        CL = CLs;
        CD = CDs;
    else
        CL = CLc;
        CD = CDc;
    end

    actual = 0;
    v = lsp(i);
    SA = slip_ang(i);

    RF = 0.25*(mass*9.81 + 0.5*den*A*CL*v^2);
    [~,Long,~] = tyres(camber,SA,RF,tyre_model);
    Long = 4*Long;

    full = Long/(mass*9.81);

    if Long_Accel(i)<0
        actual = abs(Long_Accel(i));    
    end   
    
    Drag = 0.5*A*CD*den*v^2;
    brakeG = actual-Drag/(mass*9.81);
    
    bb= (brakeG/full)*100;

    if bb>100
        bb =100;
    end
    if bb<0
        bb=0;
    end

    brake_graph(i) = bb; 
    

end
end