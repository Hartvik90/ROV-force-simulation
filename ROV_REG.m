clear all 
close all
clc
%%

% Laget av          : Hartvik Line og Olav Husebye Karstensen
% DATO              : (01.01.2016)
% Revision          : 1_0 (17.04.2015)
% Utviklet på       : MATLAB (R2015a)
% Filnavn           : ROV_REG.m
% Tilhørende filer  : ROV_REG.m, ROV_REG_SIM.slx, easyarrow.m



% Hastighetsvektor vt beskriver fartøyets hastighet i sitt koordinatsystem låst til fartøyet, bff. se figur ***.
u = 0;
v = 0;
w = 0;
p = 0;
q = 0;
r = 0;
vb1 =   [ u, v, w];             %	Translatorisk hastighet u: surge, v: sway, w: heave                 [m/s]
vb2 =   [p, q, r];              %	Vinkelhastighet p: roll, q: pitch, r: yaw                       [rad/s]
vt =    [vb1, vb2].';           %   Summerer hastighet og vinkelhastighet i en transpondert vektor

% Posisjonsvektor nt beskriver avstanden mellom NED koordinatsystem og fartøyets aksesystem, samt vinkelen mellom de
x       = 0;
y       = 0;
z       = 0;
roll    = 0;
pitch   = 0;
yaw     = 0;
n1 = [x, y, z]                  %	Posisjon	translatorisk distanse                          [m]
n2 = [roll, pitch, yaw]         %	Vinkel		roll, pitch, yaw                                [rad]
ntInit = [n1, n2].';                %   Summerer posisjon og vinkel i en transpondert vektor
%% Fysikkparametre

Mass = 17       %Vekt ROV, i kg
rho = 1000      %Tetthet, vann kg/M^3

%Posisjon av thrustere
%Beskriver avstanden til thrusterene i fartøyets koordinatsystem.
TVpX = [0.120, -0.120, -0.120 0.120]
TVpY = [0.091, 0.091, -0.091 -0.091]
TVpZ = [0.105, 0.105, 0.105, 0.105]
THpX = [0.175, -0.175, -0.175 0.175]
THpY = [0.170, 0.170, -0.170, -0.170]
THpZ = [0.000, 0.000, 0.000, 0.000]


TX = [TVpX, THpX];
TY = [TVpY, THpY];
TZ = [TVpZ, THpZ];

%Vinkel til thrustere 5:8
%%TODO! Finne riktige vinkelmål, alfa beta gamma!
ThDir1 = [0, 0, 0]
ThDir2 = [0, 0, 0]
ThDir3 = [0, 0, 0]
ThDir4 = [0, 0, 0]
ThDir5 = [-pi/4, (-3*pi)/4 ,0]
ThDir6 = [pi/4, -pi/4 ,0]
ThDir7 = [-pi/4, (-3*pi)/4 ,0]
ThDir8 = [pi/4, -pi/4 ,0]
ThDir = [ThDir1;ThDir2;ThDir3;ThDir4;ThDir5;ThDir6;ThDir7;ThDir8]'


%Kraftvirkning på aksekorset bff (BODY FIXED FRAME). Både mhp translasjon og rotasjon
PT = [0,0,0,0,cos(ThDir(1,5:8))
      0,0,0,0,cos(ThDir(2,5:8))
      cos(ThDir(3,5:8)),0,0,0,0
      TY(1),TY(2),TY(3),TY(4),0,0,0,0
      -TX(1),-TX(2),-TX(3),-TX(4),0,0,0,0
      0,0,0,0,-sqrt(TX(5)^2+TY(5)^2),-sqrt(TX(6)^2+TY(6)^2),sqrt(TX(7)^2+TY(7)^2),sqrt(TX(8)^2+TY(8)^2)
     ]
 
 
 %Parametre for dragkoefisient
 rhoDB = rho/1000   %kg/l
 ADB = 0.5          %Areal mot kraftretning
 DDB = 0.8          %Dragkoefisient
%% Plotter kraftretningene til thrusterene i med romkoordinater
figure(1);
plot3(TX,TY,TZ,'*')
xlabel('X -->')
ylabel('Y -->')
zlabel('Z -->')
for i = 1:8
easyarrow(TX(i),TX(i)+(PT(1,i)*0.2),TY(i),TY(i)+(PT(2,i)*0.2),TZ(i),TZ(i)+PT(3,i)*0.2)  
end
easyarrow(0,0.5,0,0,0,0,'solid', 0,'linewidth',3)
%text(-0.1,0,0,'ROV')


% Matrise med kraftpåvirkning på bff mhp translation and rotation
X = 0;
Y = 0;
Z = 0;
K = 0;
N = 0;
M = 0;
FullRot = 0;
CF = [X;Y;Z;K;M;N;FullRot];



%% FT = Pådragsvektor

ThrusterGain = [0;0;0;0;0;0;0;0];

% Kraftpåvirkning i hydrodynamisk modell: ROV/AUV (Ref: Fosen)
% Sum krefter = 0. -> M*V' + C(v)*v + D(v)*v + g(n) = FT
% Hvor:
% M*V': Akselerasjonskrefter (inertia)
% FT: Setpunkt for pådrag til thrusterene
% C(vt) Beskriver
% D(vt) Beskriver dempingen fra vann samt friksjon (potential damping,
% viscous damping and skin friction)
% g (nt) beskriver kreftene som kommer fra tyngdekraft og buoyancy.



%GainThrusters = PT' * RegSet




C = [0, 0, 0, 0]
