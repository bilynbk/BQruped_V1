%% (a) Initial Setup
% Define the necessary directories, so that all library-files of the
% framework and the correct model files can be accessed.  Retrieve all
% initial model states and parameters from the state definition files.

% Make a clean sweep:
clear all
close all
clc
% Define a base directory to make this file easily portable to other computers:
GaitCreationDir = 'F:\GaitCreation_SLIP\gaitcreation';
if ~exist(GaitCreationDir,'dir')
    error('SummerschoolSLIP:Main:GaitCreationDirectorNotExist', 'The specified GaitCreation-directory was not found on your computer.  Please adjust this path to match the installiation on your computer')
end
if (isunix)
    slash = '/';
else
    slash = '\';
end
cd([GaitCreationDir, slash, 'Models', slash, 'Summerschool - SLIP running']);
% Reset the MATLAB search path to its default value:
path(pathdef);
% Set the path to include all library functions:
path(path,[GaitCreationDir,slash,'Shared;',...
           GaitCreationDir,slash,'Shared',slash,'Analysis;',...
           GaitCreationDir,slash,'Shared',slash,'Graphics',slash,'Misc;',...
           GaitCreationDir,slash,'Shared',slash,'Graphics',slash,'SeriesElasticActuation;',...
           GaitCreationDir,slash,'Shared',slash,'Graphics;',...
           GaitCreationDir,slash,'Shared',slash,'Utilities;',...
           GaitCreationDir,slash,'Shared',slash,'Synthesis;']);
% Set the path to include the model specific functions:
% (Every time a different model is processed, it is important to check that
% the path only includes the directories of the current model)
path(path,[GaitCreationDir,slash,'Models',slash,'Summerschool - SLIP running;',...
           GaitCreationDir,slash,'Models',slash,'Summerschool - SLIP running',slash,'Dynamics;',...
           GaitCreationDir,slash,'Models',slash,'Summerschool - SLIP running',slash,'Dynamics',slash,'Definitions;',...
           GaitCreationDir,slash,'Models',slash,'Summerschool - SLIP running',slash,'Graphics;']);

% Get the basic state and parameter values, their names, and the
% corresponding index mapping.  By this we can access the vectors by name,
% which keeps the structure very general but allows a clear indexing.
[contStateVec, contStateNames, contStateIndices] = ContStateDefinition();
[discStateVec, discStateNames, discStateIndices] = DiscStateDefinition();
[systParamVec, systParamNames, systParamIndices] = SystParamDefinition();


%% (b) Basic simulation with various output-options
% Define a set of initial state and parameter vectors, and call the basic
% simulation functionality.  Various output options are introduced.

% Initial values for continuous and discrete states, as well as for the
% system parameters are copied from the basic state definitions: 
yIN = ContStateDefinition;
zIN = DiscStateDefinition;
p   = SystParamDefinition;
% Define additional options for the simulation:
simOptions.tIN  = 0;  % The simulation will start at t = 0
simOptions.tMAX = 5;  % The simulation will abort when t reaches 5.  This prevents an infinite simulation-loop, when the terminal event is missed.

% Simulate one full stride (which ends when the swing foot strikes the ground)
[yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, simOptions);

% Some more output options:
    % The states of both simulations are now plotted while simulating.  The
    % class is created with the flag 'false', as it's simulating a purely
    % passive system:  
    plotOUTPUT = PlotStateCLASS(false);
    HybridDynamics(yIN, zIN, p, plotOUTPUT, simOptions);
    
    % Similarly, we can record the states for later use:
    recOUTPUT = RecordStateCLASS();
    [yOUT, zOUT, tOUT, recOUTPUT] = HybridDynamics(yIN, zIN, p, recOUTPUT, simOptions);
    simRES = recOUTPUT.retrieve();
    figure('Name','SLIP model: y and dy of in-place hopping','WindowStyle','docked')
    grid on; hold on; box on;
    % Define which states are plotted:
    plotStates = [contStateIndices.y , contStateIndices.dy];
    plot(simRES.t,simRES.continuousStates(plotStates,:))
    legend(simRES.continuousStateNames(plotStates));
    
    % A graphical output can be linked as well:
    graphOUTPUT = SLIP_Model_Graphics(p);
    HybridDynamics(yIN, zIN, p, graphOUTPUT, simOptions);

% Let's add some forward motion to the initial states:
yIN(contStateIndices.dx) = 1;
[yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, graphOUTPUT, simOptions);

% Since this is obviously not periodic, we need to change the angle of
% attack:
p(systParamIndices.angAtt) = 0.3;  % Gives a roughly periodic motion
graphOUTPUT = SLIP_Model_Graphics(p); % Must be called again with new parameters p, such that the new angle of attack is visualized
[yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, graphOUTPUT, simOptions);