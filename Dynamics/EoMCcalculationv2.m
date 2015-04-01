%script EoMCalculation
%
% This script performs computation of Equation of motion of the planar
% Quadruped Robot with a flexibly spinal joint and asymmetric body-mass distribution
%
%
%%%%%%%%%%%%

clear all
clc
%% Definitions
% Generalized coordinates, initial values thetaB0 thetaF0
syms xG zG thetaB thetaF thetaB0 thetaF0
q = [xG;zG;thetaB;thetaF]

%Generalized speeds
syms dxG dzG dthetaB dthetaF
dq = [dxG;dzG;dthetaB;dthetaF]

%Generalized acceleration
syms ddxG ddzG ddthetaB ddthetaF
ddq = [ddxG;ddzG;ddthetaB;ddthetaF]

% Define necessary parameter for EoM- nondimensional params

% Inertias of back body and front body
syms IB IF

%spring stiffness of spine and pair of back legs, pair of front legs
syms kS kB kF

%back body length and front body length
syms lB lF

%knee joint angle of pair of back legs and those of pair of front legs
%betaBL = betaBR = betaB, betaFL = betaFR = betaF. initial values are
%betaB0 betaF0
%syms betaB0 betaF0

% Froud number
syms  Fr

%attack angle for both front and back legs
syms alphaB alphaF

%ground index (id)
%id = 0: flight phase, id = 1: stance phase, idB stands for back legs, idF
%stands for front legs
syms idB idF

% lower segment length lD, upper segment length lU, lU = 0.7*lD
syms lD
u = [IB IF kB kF kS lB lF lD alphaB alphaF Fr];

%% Dynamics Computation- Equation of motion-Lagrange Equation
% position of back and front body
xB = xG-(lF/(lB+lF))*(lB*cos(thetaB)+lF*cos(thetaF));
zB = zG-(lF/(lB+lF))*(lB*sin(thetaB)+lF*sin(thetaF));
xF = xG+(lB/(lB+lF))*(lB*cos(thetaB)+lF*cos(thetaF));
zF = zG+(lB/(lB+lF))*(lB*sin(thetaB)+lF*sin(thetaF));

%velocity of back and front body(using jacobian)
dxB = jacobian(xB,q)*dq;
dzB = jacobian(zB,q)*dq;
dxF = jacobian(xF,q)*dq;
dzF = jacobian(zF,q)*dq;

%acceleration of back and front body(using jacobian)
ddxB = jacobian(dxB,q)*dq+ jacobian(dxB,dq)*ddq;
ddzB = jacobian(dzB,q)*dq+ jacobian(dzB,dq)*ddq;
ddxF = jacobian(dxF,q)*dq+ jacobian(dxF,dq)*ddq;
ddzF = jacobian(dzF,q)*dq+ jacobian(dzF,dq)*ddq;

% leg property
% leg angle is calculated by using cosin law, related to segment length,
% lower segment length lD, upper segment length lU, lU = 0.7*lD
%l^2 = 1.49*lD^2-1.4*lD*cos(beta)--> cos(beta) = (1.49*lD^2-l^2)/(1.4*lD^2); l^2 = (xBH-xBFoot)^2+zBH^2; xBH-xBFoot = lB*cos(alphaB)   
xBH = xB - lB*cos(thetaB);
zBH = zB - lB*sin(thetaB);
xFH = xF + lF*cos(thetaF);
zFH = zF + lF*sin(thetaF);

betaB = acos((1.49*lD^2-cos(alphaB)^2-(zB-lB*sin(thetaB))^2)/(1.4*lD^2));
betaF = acos((1.49*lD^2-cos(alphaF)^2-(zF-lF*sin(thetaF))^2)/(1.4*lD^2));
beta0 = acos((1.49*lD^2-1)/(1.4*lD^2));
betaB0 = beta0;
betaF0 = beta0;

%kinetic energy
T = 1/2*((lB/(lB+lF))*(dxB^2+dzB^2)+(lF/(lB+lF))*(dxF^2+dzF^2)+IB*dthetaB^2+IF*dthetaF^2);
T = simple(factor(T));

%potential energy
V =  1/Fr*((lB*zB+lF*zF)/(lB+lF)+1/2*(2*idB*kB*(betaB-betaB0)^2+2*idF*kF*(betaF-betaF0)^2+kS*(-thetaF+thetaB+thetaF0-thetaB0)^2));
V = simple(factor(V));

% Lagrangian
L = T-V;

% partial derivatives computed by jacobian
dVdq = jacobian(V,q).';
dLdq = jacobian(L,q).';

dLdqdt = jacobian(L,dq).';
d_dLdqdt_dt =  jacobian(dLdqdt,q)*dq+ jacobian(dLdqdt,dq)*ddq;

LG = simple(factor(d_dLdqdt_dt - dLdq));
eqn1 = LG(1) == 0.0;
eqn2 = LG(2) == 0.0;
eqn3 = LG(3) == 0.0;
eqn4 = LG(4) == 0.0;
ddqsol = solve([eqn1,eqn2,eqn3,eqn4],ddxG,ddzG,ddthetaB,ddthetaF);

ddthetaBsol = simplify(ddqsol.ddthetaB);
ddthetaFsol = simplify(ddqsol.ddthetaF);

ddxGsol = char(ddqsol.ddxG);
ddzGsol = char(ddqsol.ddzG);
ddthetaBsol = char(ddthetaBsol);
ddthetaFsol = char(ddthetaFsol);

fileID = fopen('ddthetaBFv2.txt','w');
fprintf(fileID,'%s\n\n\n\n\n\n %s\n\n\n\n\n\n %s\n\n\n\n\n\n %s',ddxGsol,ddzGsol,ddthetaBsol,ddthetaFsol);
fclose(fileID);


%% Create Matlab functions
%BoundingDyna = 'D:\BoundingQuadruped\Models\BoundingQuadruped_V1\Dynamics';
%if ~exist([BoundingDyna,'\AutoGenFuncts'],'dir')
%    mkdir([BoundingDyna,'\AutoGenFuncts'])
%end

%matlabFunction(T,'file','AutoGenFuncts\KineticEner','vars',[dq, u]);
