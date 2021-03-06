% *************************************************************************
% 
% function [discStateVec, discStateNames, discStateIndices] = DiscStateDefinition()
% function z = DiscStateDefinition()
%
% This MATLAB function defines the discrete state vector 'z' for a
% bounding quadruped in 2D.  Besides serving as initial configuration of
% the model, this file provides a definition of the individual components
% of the discrete state vector and an index struct that allows name-based
% access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial discrete states as the vector 'discStateVec' (or 'z')
%         - The corresponding state names in the cell array 'discStateNames' 
%         - The struct 'discStateIndices' that maps these names into indices  
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
%            CONTSTATEDEFINITION, SYSTPARAMDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [discStateVec, discStateNames, discStateIndices] = DiscStateDefinition()
    
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    discState.phaseF = 2; % ['1','2'] The current phase of the front leg (stance = 1) (flight = 2)
    discState.phaseB = 2; % ['1','2'] The current phase of the back leg (stance = 1) (flight = 2)
    discState.COT    = 0; % [m_0*g]     Cost of transportation (posWork/distance traveled)
    
    [discStateVec, discStateNames] = Struct2Vec(discState);
    discStateIndices = Vec2Struct(1:1:length(discStateVec),discStateNames);
end
