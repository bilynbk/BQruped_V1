% *************************************************************************
% 
% function [CoGs, links, footPts] = GraphicalKinematicsWrapper(y, p)
%
% This MATLAB function calls the automatically generated function code that
% computes the positions of all CoGs, links, and the foot points, to
% facilitate graphical output.  This function is not called directly to
% reflect the definition of the continuous states 'y'.  It is written for
% the model of a bounding quadruped. 
%
% Input:  - A vector of continuous states 'y'
%         - A vector of model system parameters 'p'
% Output: - The position and orientation (in rows of [x;y;ang]) of all CoGs
%         - A series of line segments, that represent the links of the
%           robot
%         - All foot points of the robot.
%
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
%   See also SYMBOLICCOMPUTATIONOFEQM, CONTSTATEDEFINITION,
%            SYSTPARAMDEFINITION. 

function [CoGs, links, footPts] = GraphicalKinematicsWrapper(y, p)
    % Map the generalized coordinates:
    % Keep the index-structs in memory to speed up processing
    persistent contStateIndices
    if isempty(contStateIndices)
        [~, ~, contStateIndices] = ContStateDefinition();
    end
    x       = y(contStateIndices.x);
    y_      = y(contStateIndices.y);
    phi     = y(contStateIndices.phi);
    alphaF  = y(contStateIndices.alphaF);
    lF      = y(contStateIndices.lF);
    alphaB  = y(contStateIndices.alphaB);
    lB      = y(contStateIndices.lB);
    
    % Map the system paramters:
    % Keep the index-structs in memory to speed up processing
    persistent systParamIndices
    if isempty(systParamIndices)
        [~, ~, systParamIndices] = SystParamDefinition();
    end
    l1    = p(systParamIndices.l1);
    l2    = p(systParamIndices.l2);
    l3    = p(systParamIndices.l3);
    rFoot = p(systParamIndices.rFoot);
    g     = p(systParamIndices.g);
    m1    = p(systParamIndices.m1);
    m2    = p(systParamIndices.m2);
    m3    = p(systParamIndices.m3);
    j1    = p(systParamIndices.j1);
    j2    = p(systParamIndices.j2);
    j3    = p(systParamIndices.j3);
    
    % Call the auto-generated functions
    CoGs    = CoGPositions(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3);
	links   = LinkPositions(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3);
	footPts = FootPtPositions(x,y_,phi,alphaF,lF,alphaB,lB,l1,l2,l3,rFoot,g,m1,m2,m3,j1,j2,j3);
end
