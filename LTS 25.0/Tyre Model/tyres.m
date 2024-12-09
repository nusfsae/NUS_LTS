function [Lat,Long,GH] = tyres(camber,alpha,Reaction_f,tyre_model)

Reaction_f = Reaction_f/2;
CS = tyre_model.CS(camber,Reaction_f);
S_H = tyre_model.S_H(camber,Reaction_f);
S_V = tyre_model.S_V(camber,Reaction_f);
mu = tyre_model.mu(camber,Reaction_f);
B = tyre_model.B(camber,Reaction_f);
E =  tyre_model.E(camber,Reaction_f);

a = (CS/Reaction_f)./mu.*(tan((alpha-S_H)*pi/180));
F_bar = sin((1/B).*atan(B.*(1-E).*a+E.*atan(B.*a)));
F_shift = F_bar*mu*Reaction_f;
Lat = (F_shift+S_V);
Long = (sqrt((mu*Reaction_f).^2-Lat.^2))*2;
Lat = Lat.*2;
GH = CS;
end

