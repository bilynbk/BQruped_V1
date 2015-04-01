% *************************************************************************
%
% function [f_diff, F_lF, T_alphaF, F_lB, T_alphaB] = ComputeDifferentiableForces(y,u,p)
%
% 
% This MATLAB function computes the vector of differentiable forces for
% a bounding quadruped in 2D.  I.e. the sum of gravitational forces,
% coriolis forces, and the forces in the actuator springs. The model's
% current continuous states and the model parameters are provided by the
% calling routine to which the differentiable force vector is returned.   
% 
%
% Input:  - A vector of continuous states 'y' 
%         - A vector of excitation states 'u' 
%         - A vector of model system parameters 'p'
%
% Output: - The differentiable force-vector 'f_diff' 
%         - The leg actuator forces 'F_lF' and 'F_lB'
%         - The hip actuator torques 'T_alphaF' and 'T_alphaB'
%
% Created by C. David Remy on 03/14/2011
% MATLAB 2010a
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
% (1) Autonomous Systems Lab, Institute of Robotics and Intelligent Systems, 
%     Swiss Federal Institute of Technology (ETHZ) 
%     Tannenstr. 3 / CLA-E-32.1
%     8092 Zurich, Switzerland  
%     cremy@ethz.ch; rsiegwart@ethz.ch
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%     buffintk@bucknell.edu
%
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPSET,
%            CONTSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTSTATEDEFINITION. 
%
function [f_diff, F_lF, T_alphaF, F_lB, T_alphaB] = ComputeDifferentiableForces(y, u, p)
    
    % Get a mapping for the state and parameter vectors.
    % Keep the index-structs in memory to speed up processing
    persistent contStateIndices systParamIndices exctStateIndices
    if isempty(contStateIndices) || isempty(systParamIndices) || isempty(exctStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
        [~, ~, systParamIndices] = SystParamDefinition();
        [~, ~, exctStateIndices] = ExctStateDefinition();
    end
    
    % Compute the viscous damping coefficient of the spring, according to the desired
    % damping ratio:
    j_leg   = p(systParamIndices.j3) + (p(systParamIndices.l_0) - p(systParamIndices.l3))^2*p(systParamIndices.m3) + p(systParamIndices.j2) + p(systParamIndices.l2)^2*p(systParamIndices.m2); % total leg inertia wrt the hip
    balpha  = p(systParamIndices.balphaRat)*2*sqrt(p(systParamIndices.kalpha)*j_leg); 
    bl      = p(systParamIndices.blRat)*2*sqrt(p(systParamIndices.kl)*p(systParamIndices.m3)); 
    
    % Compute spring and damping forces:
    % Forces front leg:
    F_lF     = p(systParamIndices.kl)*(p(systParamIndices.l_0) + u(exctStateIndices.ulF)  - y(contStateIndices.lF)) + ...
               bl*(                                            + u(exctStateIndices.dulF) - y(contStateIndices.dlF));
    T_alphaF = p(systParamIndices.kalpha)*(p(systParamIndices.alphaF_0) + u(exctStateIndices.ualphaF)  - y(contStateIndices.alphaF)) + ...
               balpha*(                                                 + u(exctStateIndices.dualphaF) - y(contStateIndices.dalphaF));
    % Forces back leg:
    F_lB     = p(systParamIndices.kl)*(p(systParamIndices.l_0) + u(exctStateIndices.ulB)  - y(contStateIndices.lB)) + ...
               bl*(                                            + u(exctStateIndices.dulB) - y(contStateIndices.dlB));
    T_alphaB = p(systParamIndices.kalpha)*(p(systParamIndices.alphaB_0) + u(exctStateIndices.ualphaB)  - y(contStateIndices.alphaB)) + ...
               balpha*(                                                 + u(exctStateIndices.dualphaB) - y(contStateIndices.dalphaB));
    
     % Graviational and coriolis forces:
    f_act = [0; 0; 0; T_alphaF; F_lF; T_alphaB; F_lB];
    % Graviational and coriolis forces:
    f_cg = F_CoriGravWrapper(y,p);
    % All differentiable forces:
    f_diff = f_cg + f_act;
end
