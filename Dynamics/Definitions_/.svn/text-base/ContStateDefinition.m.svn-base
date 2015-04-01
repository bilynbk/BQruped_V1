% *************************************************************************
% 
% function [contStateVec, contStateNames, contStateIndices] = ContStateDefinition()
% function y = ContStateDefinition()
%
% This MATLAB function defines the continuous state vector 'y' for a
% bounding quadruped in 2D.  Besides serving as initial configuration of
% the model, this file provides a definition of the individual components
% of the continuous state vector and an index struct that allows name-based
% access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial continuous states as the vector 'contStateVec' (or 'y')
%         - The corresponding state names in the cell array 'contStateNames' 
%         - The struct 'contStateIndices' that maps these names into indices  
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
%            DISCSTATEDEFINITION, SYSTPARAMDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [contStateVec, contStateNames, contStateIndices] = ContStateDefinition()
    
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    contState.x       =  0.00;       % [l_0] horizontal position of the main body CoG
    contState.dx      =  0.70;       % [sqrt(g*l_0)] ... velocity thereof
    contState.y       =  1.15;       % [l_0] vertical position of the main body CoG
    contState.dy      =  0.00;       % [sqrt(g*l_0)] ... velocity thereof
    contState.phi     =  0.01;       % [rad] angle of the main body wrt horizontal (pitch)
    contState.dphi    =  0.15;       % [rad/sqrt(l_0/g)] ... angular velocity thereof
    contState.alphaF  = -0.32;       % [rad] angle of the front leg wrt the main body (0 = straight down)
    contState.dalphaF =  0.64;       % [rad/sqrt(l_0/g)] ... angular velocity thereof
    contState.lF      =  1.07;       % [l_0] length of the front leg (hip-joint to foot-center)
    contState.dlF     = -0.57;       % [sqrt(g*l_0)] ... velocity thereof
    contState.alphaB  =  0.01;       % [rad] angle of the back leg wrt the main body (0 = straight down)
    contState.dalphaB =  0.89;       % [rad/sqrt(l_0/g)] ... angular velocity thereof
    contState.lB      =  0.97;       % [l_0] length of the back leg (hip-joint to foot-center)
    contState.dlB     =  0.10;       % [sqrt(g*l_0)] ... velocity thereof
    contState.time    =  0.00;       % [sqrt(l_0/g)] time that has passed since the start of the step
    contState.posWork =  0.00;       % [m_0*g*l_0] positive mechanical work of the actuators
    
    [contStateVec, contStateNames] = Struct2Vec(contState);
    contStateIndices = Vec2Struct(1:1:length(contStateVec),contStateNames);
end
