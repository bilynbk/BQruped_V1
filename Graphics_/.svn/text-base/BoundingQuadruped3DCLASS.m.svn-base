% *************************************************************************
%
% classdef BoundingQuadruped3DCLASS(full3D) < OutputCLASS
%
% Three dimensional graphics of a bounding quadruped with series elastic
% actuation.  In the graphical representation, the robot is drawn with four
% legs of which two always move together.
%
% The system is initialized with a Boolean flag that indicates whether the
% view is fully three dimensional, or rather flat from the side.  The
% motion of the model is, however, in both cases 2D. 
%
%
% Properties: - NONE
% Methods:    - NONE
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
%   See also OUTPUTCLASS.
%
classdef BoundingQuadruped3DCLASS < OutputCLASS 
    properties (SetAccess = 'private', GetAccess = 'private')
        fig; % The output window
        % Patch objects used in the 3D representation
        QuadrupedPatch;
        FloorPatch;
        % Vertices to which the transformation will be applied
        FloorVertices;
        % View Flag
        full3D
    end
    properties 
    end
    methods
        function obj = BoundingQuadruped3DCLASS(full3D)
            obj.slowDown = 1; % Run this in real time.
            obj.rate     = 0.04;   % with 25 fps
            obj.full3D   = full3D;
            % Initialize the 3D graphics
            obj.fig = figure();
            clf(obj.fig);
            % Set some window properties
            set(obj.fig,'Name','3D-Output of a bounding quadruped');  % Window title
            set(obj.fig,'Color','w');         % Background color
            
            %% Create the 3D Objects:  
            % Create the floor:
            if full3D
                [f,v,c] = Get3DGroundPatch();
                p = patch('faces', f, 'vertices', v, 'FaceVertexCData', c, 'FaceColor', 'flat');
            	set(p, 'FaceAlpha',0.7);    % Set to partly transparent
            else
                [f,v,c] = Get2DGroundPatch();
                p = patch('faces', f, 'vertices', v, 'FaceVertexCData', c, 'FaceColor', 'flat');
            end
            set(p, 'FaceLighting','phong'); % Set the renderer
            set(p, 'FaceColor','flat');     % Set the face coloring
            set(p, 'EdgeColor','none');     % Don't show edges
            % Store theses objects for later use (the ground must be
            % shifted later, if the hopper moves)
            obj.FloorPatch    = p;
            obj.FloorVertices = v;
            % Create the bounding robot (with some arbitrary initial condition)
            [f,v,c] = GetQuadrupedPatch(0, 1.2, 0.1, 0.2, 0, 0.1, 0, 1, 0, 1, 0);
            p = patch('faces', f, 'vertices', v, 'FaceVertexCData', c, 'FaceColor', 'flat');
            set(p, 'FaceLighting','phong');  % Set the renderer
            set(p, 'FaceColor','flat');      % Set the face coloring
            set(p, 'EdgeColor','none');      % Don't show edges
            obj.QuadrupedPatch = p;
            %% set up view:
            axis off
            box off
            axis equal
            if full3D
                camproj('perspective');
                camtarget([0, 0, +0.6])
                campos ([5, -10, +2.6]);
                camup ([0,0,1]);
                camva(15)
            else
                camproj('orthographic');
                camtarget([0, 0, +0.6])
                campos ([0, -10, +0.6]);
                camup ([0,0,1]);
                camva(15)
            end
            %% Create illumination:
            light('Position',[5 -10 12 ],'Style','local');   % parallel light, coming in the direction given in 'Position'
        end
        function obj = update(obj, y, ~, ~, u)
            persistent contStateIndices exctStateIndices
            if isempty(contStateIndices) || isempty(exctStateIndices)
                [~, ~, contStateIndices] = ContStateDefinition();
                [~, ~, exctStateIndices] = ExctStateDefinition();
            end
            % The floor is shifted to multiples of its pattern, so it's
            % not noticeable in the final graphics: 
            v = TransformVertices(obj.FloorVertices,...
                                  diag([1,1,1]),...
                                  [floor(y(contStateIndices.x)/2)*2,0,-0.05]);
            set(obj.FloorPatch,'Vertices',v);
            % The quadruped:
            [~, v] = GetQuadrupedPatch(y(contStateIndices.x),...
                                       y(contStateIndices.y),...
                                       y(contStateIndices.phi),...
                                       y(contStateIndices.alphaF),...
                                       u(exctStateIndices.ualphaF),...
                                       y(contStateIndices.alphaB),...
                                       u(exctStateIndices.ualphaB),...
                                       y(contStateIndices.lF),...
                                       u(exctStateIndices.ulF),...
                                       y(contStateIndices.lB),...
                                       u(exctStateIndices.ulB));
            set(obj.QuadrupedPatch,'Vertices',v); 
            % Set camera:
            if obj.full3D
                camtarget([y(contStateIndices.x), 0, +0.6])
                campos ([5, -10, +2.6]);
            else
                camtarget([y(contStateIndices.x), 0, +0.6])
                campos ([y(contStateIndices.x), -10, +0.6]);
            end
            drawnow();
        end
    end
end