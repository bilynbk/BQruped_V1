function cont_pointB = ContactPointB(x,y,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3)
%CONTACTPOINTB
%    CONT_POINTB = CONTACTPOINTB(X,Y,PHI,ALPHAF,LF,ALPHAB,LB,L1,L2,L3,RFOOT,G,M1,M2,M3,J1,J2,J3)

%    This function was generated by the Symbolic Math Toolbox version 5.4.
%    10-Apr-2011 10:49:22

t676 = alphaB+phi;
cont_pointB = [x-l1.*cos(phi)+lB.*sin(t676);-rFoot+y-lB.*cos(t676)-l1.*sin(phi)];
