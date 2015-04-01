% *************************************************************************
% 
% function [systParamVec, systParamNames, systParamIndices] = SystParamDefinition()
% function p = SystParamDefinition()
%
% This MATLAB function defines the physical system parameter vector 'p' for
% a bounding quadruped in 2D.  Besides serving as initial configuration of
% the model, this file provides a definition of the individual components
% of the system parameter vector and an index struct that allows name-based
% access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial system parameters as the vector 'systParamVec' (or 'p')
%         - The corresponding parameter names in the cell array 'systParamNames' 
%         - The struct 'systParamIndices' that maps these names into indices  
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPMAP, JUMPSET, 
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [systParamVec, systParamNames, systParamIndices] = SystParamDefinition()
    
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    % Physics:
    systParam.g         = 1;     % [g] gravity
    % Parameter of the model
    systParam.l_0       = 1;     % [l_0] uncompressed leg length
    systParam.alphaF_0  = 0;     % [rad] resting front leg angle
    systParam.alphaB_0  = 0;     % [rad] resting back leg angle
    systParam.m1        = 0.70;  % [m_0] mass of the main body
    systParam.m2        = 0.10;  % [m_0] mass of the upper leg segments
    systParam.m3        = 0.05;  % [m_0] mass of the lower leg segments
    systParam.l1        = 0.75;  % [l_0] distance between CoG of the main body and hip joints
    systParam.l2        = 0.25;  % [l_0] distance between hip joints and CoG of the upper leg segments
    systParam.l3        = 0.25;  % [l_0] distance between foot points and CoG of the lower leg segments
    systParam.rFoot     = 0.05;  % [l_0] foot radius
    systParam.j1        = 0.4;   % [m_0*l_0^2] inertia of the main body
    systParam.j2        = 0.002; % [m_0*l_0^2] inertia of the upper leg segments
    systParam.j3        = 0.002; % [m_0*l_0^2] inertia of the lower leg segments
    systParam.kalpha    = 5;     % [m_0*g*l_0/rad] rotational spring stiffness in the hip joints
    systParam.balphaRat = 0.2;   % [*] damping ratio.  Use 20% of critical damping
    systParam.kl        = 10;    % [m_0*g/l_0] linear spring stiffness in the prismatic joints
    systParam.blRat     = 0.2;   % [*] damping ratio.  Use 20% of critical damping
    
    [systParamVec, systParamNames] = Struct2Vec(systParam);
    systParamIndices = Vec2Struct(1:1:length(systParamVec),systParamNames);
end
