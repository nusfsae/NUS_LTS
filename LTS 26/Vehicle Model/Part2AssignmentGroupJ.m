%% ME4233 Assignment 1 Part 2

% Dai Baizhou A0266515B
% Sam Marret A0252323R
% Aditiya A0244516H

%% (a)
% C value
C = mean([26,025,024]);
% initialize
Nx = 60; Ny = 50;
Lx = 1; Ly = 1;
dx = Lx/Nx; dy = Ly/Ny;
Gamma = dx/dy; D = -2*(Gamma^2+1);
% empty grid
x = dx:dx:dx*(Nx-1);
y = dy:dy:dy*(Ny-1);
% form matrix A
aux1 = toeplitz([D 1 zeros(1,Nx-3)],[D 1 zeros(1,Nx-3)]');
aux2 = Gamma^2*eye(Nx-1);
A=[aux1 aux2 zeros(Nx-1,(Ny-3)*(Nx-1))];
for i=1:Ny-3
    A = [A
         zeros(Nx-1,(i-1)*(Nx-1)) aux2 aux1 aux2 zeros(Nx-1,(Ny-i-3)*(Nx-1))];
end
A=[A
   zeros(Nx-1,(Ny-3)*(Nx-1))  aux2 aux1];
% Form the vector b in Au=b
b = 1*dx^2*ones((Nx-1)*(Ny-1),1);
% Boundary conditions for b 
bc=reshape([zeros(Nx-2,Ny-1);2*ones(1,Ny-1)],[],1);
b=b-bc;
% Solve the linear problem Au=b 
u = A\b; 

%% (b)
