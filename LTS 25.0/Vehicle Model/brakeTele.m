function sim = brakeTele(sim,dist,mass,den,A,CLs,CLc,CDs,CDc,camber,C2,tyre_model,P,tc_long)

simLong = longGTele(sim,dist);
Long_Accel = simLong.longG;
len = length(sim.speed);
P = convpres(P, 'psi', 'Pa');

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
    v = sim.speed(i);
    


    Reaction_f = 0.25*(mass*9.81+0.5*den*A*CL*v^2);

    inputsMF = [Reaction_f 0.15 0 camber 0 10 P];

    [ outMF ] = mfeval(tyre_model, inputsMF, 121);
    %disp(abs(outMF(1)));
    Long = 4*tc_long*abs(outMF(1));

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

    sim.brake(i) = bb; 
    

end
end