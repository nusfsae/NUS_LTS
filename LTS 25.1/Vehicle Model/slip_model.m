function [alpha_profile] = slip_model(lsp,slip_ang,mass,air_density,frontel_area,coef_lift,camber,tyre_model,C2)
%determine alpha with estimated v, then replace v

v_profile = lsp;
alpha_profile = slip_ang;
len = length(C2);

for point = 1:len
    if alpha_profile(point,1) == 5
        disp(point)
        dif = 6;
        while dif > 1
            v = v_profile(point,1);
            alpha = alpha_profile(point,1);
            cur = C2(point);

            %disp(cur);
            %disp(v);
          

            new_alpha = slip(mass,air_density,frontel_area,coef_lift,v,camber,alpha,tyre_model,cur);
            alpha_profile(point,1) = new_alpha;
            disp(new_alpha);

            dif = abs(alpha-new_alpha);

        end
    end
end
end




