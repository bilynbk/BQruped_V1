% *************************************************************************
% 
% function [exctParamVec, exctParamNames, exctParamIndices] = ExctParamDefinition()
% function s = ExctParamDefinition()
%
% This MATLAB function defines the parameter vector 's' for the excitation
% a bounding quadruped in 2D.  Besides serving as initial configuration of
% the model, this file provides a definition of the individual components
% of the excitation parameter vector and an index struct that allows
% name-based access to its values.
%
% NOTE: This function is relatively slow and should not be executed within
%       the simulation loop.
%
% Input:  - NONE
% Output: - The initial excitation parameters as the vector 'exctParamVec' (or 's')
%         - The corresponding parameter names in the cell array 'exctParamNames' 
%         - The struct 'exctParamIndices' that maps these names into indices  
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
%            EXCTSTATEDEFINITION, 
%            VEC2STRUCT, STRUCT2VEC. 
%
function [exctParamVec, exctParamNames, exctParamIndices] = ExctParamDefinition()
    
    % All units are normalized to gravity g, total mass m_0, and
    % uncompressed leg length l_0.
    
    exctParam.strideFreq = 0.5;                   % [sqrt(g/l_0)] stride frequency. 
    exctParam.sinalphaF  = [ 0.12, 0.02,0,0,0,0,0,0,0,0]; % [rad] amplitude of the sine-terms for front leg rotation
    exctParam.cosalphaF  = [-0.27,-0.03,0,0,0,0,0,0,0,0]; % [rad] amplitude of the cosine-terms for front leg rotation
    exctParam.sinlF      = [-0.03,-0.03,0,0,0,0,0,0,0,0]; % [l_0] amplitude of the sine-terms for front leg extension
    exctParam.coslF      = [ 0.05, 0.02,0,0,0,0,0,0,0,0]; % [l_0] amplitude of the cosine-terms for front leg extension
    exctParam.sinalphaB  = [ 0.19, 0.04,0,0,0,0,0,0,0,0]; % [rad] amplitude of the sine-terms for back leg rotation
    exctParam.cosalphaB  = [ 0.10,-0.02,0,0,0,0,0,0,0,0]; % [rad] amplitude of the cosine-terms for back leg rotation
    exctParam.sinlB      = [-0.03, 0.01,0,0,0,0,0,0,0,0]; % [l_0] amplitude of the sine-terms for back leg extension
    exctParam.coslB      = [-0.01,-0.02,0,0,0,0,0,0,0,0]; % [l_0] amplitude of the cosine-terms for back leg extension
    
    [exctParamVec, exctParamNames] = Struct2Vec(exctParam);
    exctParamIndices = Vec2Struct(1:1:length(exctParamVec),exctParamNames);
end
