%s = [IB IF kB kF kS lB lF lD alphaB alphaF Fr];
S = [1.6 1.8 18 18 5. 0.3 0.45 0.7 46*pi/180 76*pi/180 4];
%q = [xG;dxG;zG;dzG;thetaB;dthetaB;thetaF;dthetaF]
Q0 = [1 1.8 0.901 0. 8*pi/180 -17*pi/180 -5*pi/180 17*pi/180];

thetaB0 = 8*pi/180;
thetaF0 = -5*pi/180;
IB = s(1);
IF = s(2);
kB = s(3);
kF = s(4);
kS = s(5);
lB = s(6);
lF = s(7);
lD = s(8);
alphaB = s(9);
alphaF = s(10);
Fr = s(11);

xG = q(1);
dxG = q(2);
zG = q(3);
dzG = q(4);
thetaB = q(5);
dthetaB = q(6);
thetaF = q(7);
dthetaF = q(8);

xB = xG-(lF/(lB+lF))*(lB*cos(thetaB0)+lF*cos(thetaF0));
zB = zG-(lF/(lB+lF))*(lB*sin(thetaB0)+lF*sin(thetaF0));
xF = xG+(lB/(lB+lF))*(lB*cos(thetaB0)+lF*cos(thetaF0));
zF = zG+(lB/(lB+lF))*(lB*sin(thetaB0)+lF*sin(thetaF0));

xBH = xB-lB*cos(thetaB0);
zBH = zB - lB*sin(thetaB0);
xFH = xF+lF*cos(thetaF0);
zFH = zF + lF*sin(thetaF0);
xS = xB + lB*cos(thetaB0);
zS = zB + lB*sin(thetaB0);

LLengthB = sqrt(cos(alphaB)^2+zBH^2);
LLengthF = sqrt(cos(alphaF)^2+zFH^2);

% chi dung khi touchdown
%tinh goc giua upper segment vs virtual leg.
Bn =acos((LLengthB^2-0.3*lD^2)/(1.4*LLengthB*lD));

%tinh toa do cua Back Leg Joint
xBJ = xBH + 0.7*lD*sin(90-alphaB+Bn);
zBJ = zBH - 0.7*lD*cos(90-alphaB+Bn);
%tinh toa do cua Back Foot
xBF = xBH + LLengthB*cos(alphaB);
zBF = zBH - LLengthB*sin(alphaB);

%tinh goc giua upper segment vs virtual leg.
Bn =acos((LLengthB^2-0.3*lD^2)/(1.4*LLengthB*lD));

%tinh toa do cua Back Leg Joint
xBJ = xBH + 0.7*lD*sin(90-alphaB+Bn);
zBJ = zBH - 0.7*lD*cos(90-alphaB+Bn);
%tinh toa do cua Back Foot
%xBF = xBH + LLengthB*cos(alphaB);
%zBF = zBH - LLengthB*sin(alphaB);
xBF = xBH + LLengthB*cos(alphaB);
zBF = zBH - LLengthB*sin(alphaB);
%--------
%tinh goc giua upper segment vs virtual leg.
Fn =acos((LLengthF^2-0.3*lD^2)/(1.4*LLengthF*lD));

%tinh toa do cua Front Leg Joint
xFJ = xFH + 0.7*lD*sin(90-alphaF+Fn);
zFJ = zFH - 0.7*lD*cos(90-alphaF+Fn);
%tinh toa do cua Front Foot
xFF = xFH + LLengthF*cos(alphaF);
zFF = zFH - LLengthF*sin(alphaF);


