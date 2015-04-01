% *************************************************************************
%
%  function u = ExcitationFunction(y, z, s)
% 
% This MATLAB file defines the excitation function h used in a bounding
% quadruped in 2D.  Its output u is depending on the continuous states 'y'
% and  discrete states 'z', as well as the excitation parameter vector 's'.
% 
%
% Input:  - A vector of continuous states 'y' 
%         - A vector of discrete states 'z' 
%         - A vector of excitation parameters 's';
%
% Output: - the states of the series series elastic driver: 'u'
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
%   See also HYBRIDDYNAMICS, FLOWMAP, JUMPSET, JUMPSET,
%            CONTSTATEDEFINITION, DISCSTATEDEFINITION, EXCTSTATEDEFINITION,
%            EXCTPARAMDEFINITION.  
%
function u = ExcitationFunction(y, ~, s)
    
    % Get a mapping for the state and parameter vectors.
    % Keep the index-structs in memory to speed up processing
    persistent exctStateIndices exctParamIndices contStateIndices; 
    if isempty(exctStateIndices) || isempty(exctParamIndices) || isempty(contStateIndices)
        [~, ~, exctStateIndices] = ExctStateDefinition();
        [~, ~, exctParamIndices] = ExctParamDefinition();
        [~, ~, contStateIndices] = ContStateDefinition();
    end
    
    % The timing variable phi is 2*pi-periodic over one stride:
    phi  = y(contStateIndices.time)*2*pi*s(exctParamIndices.strideFreq);
    dphi = 2*pi*s(exctParamIndices.strideFreq);

     % Start with a base function of 0. As no constant terms are used, the
    % average of the excitation function will remain 0 over the [0..2*pi[
    % period.
    u = zeros(8,1);
    
    % Create Fourier series by addition over all elements:
    for i = 1:length(exctParamIndices.sinlF)
        u(exctStateIndices.ulF)  = u(exctStateIndices.ulF)  +  s(exctParamIndices.sinlF(i))*sin(phi*i) + s(exctParamIndices.coslF(i))*cos(phi*i);
        u(exctStateIndices.dulF) = u(exctStateIndices.dulF) + (s(exctParamIndices.sinlF(i))*cos(phi*i) - s(exctParamIndices.coslF(i))*sin(phi*i))*dphi*i;
    end
    for i = 1:length(exctParamIndices.sinalphaF)
        u(exctStateIndices.ualphaF)  = u(exctStateIndices.ualphaF)  +  s(exctParamIndices.sinalphaF(i))*sin(phi*i) + s(exctParamIndices.cosalphaF(i))*cos(phi*i);
        u(exctStateIndices.dualphaF) = u(exctStateIndices.dualphaF) + (s(exctParamIndices.sinalphaF(i))*cos(phi*i) - s(exctParamIndices.cosalphaF(i))*sin(phi*i))*dphi*i;
    end
    for i = 1:length(exctParamIndices.sinlB)
        u(exctStateIndices.ulB)  = u(exctStateIndices.ulB)  +  s(exctParamIndices.sinlB(i))*sin(phi*i) + s(exctParamIndices.coslB(i))*cos(phi*i);
        u(exctStateIndices.dulB) = u(exctStateIndices.dulB) + (s(exctParamIndices.sinlB(i))*cos(phi*i) - s(exctParamIndices.coslB(i))*sin(phi*i))*dphi*i;
    end
    for i = 1:length(exctParamIndices.sinalphaB)
        u(exctStateIndices.ualphaB)  = u(exctStateIndices.ualphaB)  +  s(exctParamIndices.sinalphaB(i))*sin(phi*i) + s(exctParamIndices.cosalphaB(i))*cos(phi*i);
        u(exctStateIndices.dualphaB) = u(exctStateIndices.dualphaB) + (s(exctParamIndices.sinalphaB(i))*cos(phi*i) - s(exctParamIndices.cosalphaB(i))*sin(phi*i))*dphi*i;
    end
end
