%consider vehicle's limited yaw rate

function [final_lsp,yaw_diagram] = yawcal(lsp,C2,maxsteer,L)
len = length(lsp);
final_lsp = lsp;
yaw_diagram = zeros(len);
for point=1:len-1
    vel = lsp(point);
    R = 1/C2(point);

    for v = vel:-0.01:0
        maxyaw = maxsteer*v/(L+(v^2/9.81));
        yaw = v/R;

        if yaw<maxyaw
            final_lsp(point) = v; %update the maximum velocity allowed by yawing
            yaw_diagram(point) = yaw;
            break
        end
    end

end
end


