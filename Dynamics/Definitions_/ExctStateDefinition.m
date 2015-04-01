% *************************************************************************
% 
% function [exctStateVec, exctStateNames, exctStateIndices] = ExctStateDefinition()
% function u = ExctStateDefinition()
%
% This MATLAB function defines the excitation state vector 'u' for a
% bounding quadruped in 2D.  Besides serving as initial configuration of 
% the model, this file provides a definition of the individual components
% of the excitation state vector and an index struct that allows name-based
% access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The excitation states as the vector 'exctStateVec' (or 'u')
%         - The corresponding state names in the cell array 'exctStateNames'
%         - The struct 'exctStateIndices' that maps these names into indices  
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
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, SYSTPARAMDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [exctStateVec, exctStateNames, exctStateIndices] = ExctStateDefinition()
    
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    exctState.ualphaF  = 0; % [rad] motor angle of the front rotational actuator 
    exctState.dualphaF = 0; % [rad/sqrt(l_0/g)] ... velocity thereof
    exctState.ulF      = 0; % [l_0] motor position of the front linear actuator 
    exctState.dulF     = 0; % [sqrt(g*l_0)] ... velocity thereof
    exctState.ualphaB  = 0; % [rad] motor angle of the back rotational actuator 
    exctState.dualphaB = 0; % [rad/sqrt(l_0/g)] ... velocity thereof
    exctState.ulB      = 0; % [l_0] motor position of the back linear actuator 
    exctState.dulB     = 0; % [sqrt(g*l_0)] ... velocity thereof
    
    [exctStateVec, exctStateNames] = Struct2Vec(exctState);
    exctStateIndices = Vec2Struct(1:1:length(exctStateVec),exctStateNames);
end
