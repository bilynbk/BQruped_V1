clear all
close all
clc

%This MATLAB script show the passive bounding of spinal joint quadruped for
%which the optimal parameters, analysis and control is obtained.

%%
% Get the basic state and parameter values, their names, and the
% corresponding index mapping.  By this we can access the vectors by name,
% which keeps the structure very general but allows a clear indexing.
[contStateVec, contStateNames, contStateIndices] = ContStateDefinition();
[discStateVec, discStateNames, discStateIndices] = DiscStateDefinition();
[sysParamVec, sysParamNames, sysParamIndices] = SysParamDefinition();
[actStateVec, actStateNames, actStateIndices] = ActStateDefinition();
[actParamVec, actParamNames, actParamIndices] = ActParamDefinition();




%% 1) Find an optimal periodic solution for all parameters 
%INITIAL VALUES:
yINIT = contStateVec;
zINIT = discStateVec;
pINIT = sysParamVec;
sINIT = actParamVec; % just for active bounding

% VALUES OPTIMIZED
yOPTIM = ones(size(yINIT)) % optimize all values
%except horizontal starting position (should be 0), vertical velocity
%(every motion start at the apex, dy = 0), time must be 0 to be periodic,
%positive start at 0.
yOPTIM(contStateIndices.x) = 0;
yOPTIM(contStateIndices.dy) = 0;
yOPTIM(contStateIndices.time) = 0;
yOPTIM(contStateIndices.posWork) = 0;
%Initial continuous states are not altered
zOPTIMstruct = struct();
zOPTIM = Struct2Vec(zOPTIMstruct,discStateNames);





