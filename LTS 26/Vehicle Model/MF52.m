function [FY, FX] = MF52(SlipRatio,SlipAngle,NormalLoad,Camber, parameters)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 11/08/2025
% Originally from Bill Cobb - FSAE TTC Forum
% Requires adapting with MFEval generated cofficients (sanity check for
% equivalence in equations)
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Input - [SL, SA, FZ, IA] 
% Output - [FY, FX]
% Uses Coefficients Determined by Individual Lateral and Longitudinal Fitting Process
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Properties

coefficients = parameters.Tyre;

% Inputs

cf = 0.55;     % grip factor

FZ0 = coefficients.FZ0;

KAPPA = SlipRatio; % Slip Ratio [-]
ALPHA = SlipAngle; % degrees to [Rad]
FZ    = NormalLoad; % [N]
GAMMA = Camber*pi/180; % degrees to [Rad]

LFZO = coefficients.LFZO;
FZ0PR = FZ0 * LFZO; %15, NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ = (FZ-FZ0PR ) ./ FZ0PR ; %14, (%30)

% Scaling Factors

LGAY    =   coefficients.LGAY;
LHY     =   coefficients.LHY;
LVY     =   coefficients.LVY;
LCY     =   coefficients.LCY;
LEY     =   coefficients.LEY;
LHX     =   coefficients.LHX;
LVX     =   coefficients.LVX;
LGX     =   coefficients.LGX;
LCX     =   coefficients.LCX;
LEX     =   coefficients.LEX;
LXAL    =   coefficients.LXAL;

LKY     =   coefficients.LKY; % 1
LKX     =   coefficients.LKX; %  0.7
LMUY    =   cf*coefficients.LMUY; % 0.38 Changed to Fit
LMUX    =   cf*coefficients.LMUX; % 0.25 Changed to Fit

% Longitudinal Coefficients
PCX1    =   coefficients.PCX1;
PDX1    =   coefficients.PDX1;
PDX2    =   coefficients.PDX2;
PDX3    =   coefficients.PDX3;
PEX1    =   coefficients.PEX1;
PEX2    =   coefficients.PEX2;
PEX3    =   coefficients.PEX3;
PEX4    =   coefficients.PEX4;
PKX1    =   coefficients.PKX1;
PKX2    =   coefficients.PKX2;
PKX3    =   coefficients.PKX3;
PHX1    =   coefficients.PHX1;
PHX2    =   coefficients.PHX2;
PVX1    =   coefficients.PVX1;
PVX2    =   coefficients.PVX2;

% Combined Longitudinal Coefficients
RBX1    =   coefficients.RBX1;
RBX2    =   coefficients.RBX2;
RCX1    =   coefficients.RCX1;
REX1    =   coefficients.REX1;
REX2    =   coefficients.REX2;
RHX1    =   coefficients.RHX1;

% Lateral Coefficients
PCY1    =   coefficients.PCY1;
PDY1    =   coefficients.PDY1;
PDY2    =   coefficients.PDY2;
PDY3    =   coefficients.PDY3;
PEY1    =   coefficients.PEY1;
PEY2    =   coefficients.PEY2;
PEY3    =   coefficients.PEY3;
PEY4    =   coefficients.PEY4;
PKY1    =   coefficients.PKY1;
PKY2    =   coefficients.PKY2;
PKY3    =   coefficients.PKY3;
PHY1    =   coefficients.PHY1;
PHY2    =   coefficients.PHY2;
PHY3    =   coefficients.PHY3;
PVY1    =   coefficients.PVY1;
PVY2    =   coefficients.PVY2;
PVY3    =   coefficients.PVY3;
PVY4    =   coefficients.PVY4;

% Combined Lateral Coefficients
RBY1    =   coefficients.RBY1;
RBY2    =   coefficients.RBY2;
RBY3    =   coefficients.RBY3;
RCY1    =   coefficients.RCY1;
REY1    =   coefficients.REY1;
REY2    =   coefficients.REY2;
RHY1    =   coefficients.RHY1;
RHY2    =   coefficients.RHY2;
RVY1    =   coefficients.RVY1;
RVY2    =   coefficients.RVY2;
RVY3    =   coefficients.RVY3;
RVY4    =   coefficients.RVY4;
RVY5    =   coefficients.RVY5;
RVY6    =   coefficients.RVY6;


% Lateral Force

GAMMAY = GAMMA.*LGAY;
SHY = (PHY1+ PHY2.*DFZ).*LHY + PHY3.*GAMMAY;
SVY = 0; % FZ.*((PVY1 + PVY2.*DFZ).*LVY + (PVY3 + PVY4.*DFZ)).*LMUY;
ALPHAY = ALPHA + SHY;
CY = PCY1.*LCY;
MUY = (PDY1 + PDY2.*DFZ).*(1 - PDY3.*(GAMMAY.^2)).*LMUY;
DY = MUY.*FZ;
EY = (PEY1 + PEY2.*DFZ).*(1 - (PEY3 + PEY4.*GAMMAY).*sign(ALPHAY)).*LEY;
KY0 = PKY1.*FZ0.*sin( 2.*atan(FZ./(PKY2.*FZ0PR))).*LKY; % Cornering Stiffness
KVY0 = PHY3.*KY0 + FZ.*(PVY3 + PVY4.*DFZ); % Camber Stiffness
KY = KY0.*(1 - PKY3.*abs(GAMMAY));
BY = KY./(CY.*DY);
FY0 = DY.*sin(CY.*atan(BY.*ALPHAY - EY.*(BY.*ALPHAY - atan(BY.*ALPHAY)))) + SVY; % Pure Slip Lateral Force

BYK = RBY1.*cos(atan(RBY2.*(ALPHA - RBY3)));
CYK = RCY1;
EYK = REY1 + REY2.*DFZ;
SHYK = RHY1 + RHY2.*DFZ; % This kills 
KS = KAPPA + SHYK;
DVYK = MUY.*FZ.*(RVY1 + RVY2.*DFZ + RVY3.*GAMMA).*cos(atan(RVY4.*ALPHA));
SVYK = DVYK.*sin(RVY5.*atan(RVY6.*KAPPA));
DYK = FY0./( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));
GYK = ( cos(CYK.*atan( BYK.*KS - EYK.*(BYK.*KS - atan(BYK.*KS))))) ./ ( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));
FY = GYK.*FY0 + SVYK; % Combined Slip Lateral Force

% Longitudinal Force

SHX = (PHX1 + PHX2.*DFZ).*LHX;
SVX = FZ.*(PVX1 + PVX2.*DFZ).*LVX.*LMUX;
KAPPAX = KAPPA + SHX;
GAMMAX = GAMMA.*LGX;
CX = PCX1.*LCX;
MUX = (PDX1 + PDX2.*DFZ).*(1 - PDX3.*(GAMMA.^2)).*LMUX;
DX = MUX.*FZ;
EX = (PEX1 + PEX2.*DFZ + PEX3.*(DFZ.^2)).*(1 - PEX4.*sign(KAPPAX)).*LEX;
KX = FZ.*(PKX1 + PKX2.*DFZ).*exp(PKX3.*DFZ).*LKX; % Longitudinal Slip Stiffness
BX = KX./(CX.*DX);
FX0 = DX.*sin( (CX.*atan(BX.*KAPPAX - EX.*(BX.*KAPPAX - atan(BX.*KAPPAX)))) + SVX); % Pure Slip Longitudinal Force

SHXAL = RHX1;
CXAL = RCX1;
BXAL = RBX1.*cos( atan(RBX2.*KAPPA)).*LXAL; % cos term will always be positive regardless of slip ratio direction
ALPHAS = ALPHA + SHXAL;
EXAL = REX1 + REX2.*DFZ;
GXAL = ( cos(CXAL.*atan(BXAL.*ALPHAS - EXAL.*(BXAL.*ALPHAS - atan(BXAL.*ALPHAS))))) ./ ( cos(CXAL.*atan(BXAL.*SHXAL - EXAL.*(BXAL.*SHXAL - atan(BXAL.*SHXAL)))));
FX = GXAL.*FX0; % Combined Slip Slip Longitudinal Force


end