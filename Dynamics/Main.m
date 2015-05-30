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
yOPTIM = ones(size(yINIT)); % optimize all values
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
pOPTIMstruct.alphaF_0 = 1;
pOPTIMstruct.alphaB_0 = 1;
pOPTIM = Struct2Vec(pOPTIMstruct,sysParamNames);
%Find the right excitation-frequency
%sOPTIMstruct.strideFreq = 1;
%the first nrTerms(=5) pairs of Fouriers-coefficients for leg extension,
%and hip swing will be altered:
nrTerms = 5;
sOPTIMstruct.sinlF(1:nrTerms)=1;
sOPTIMstruct.coslF(1:nrTerms)=1;
sOPTIMstruct.sinalphaF(1:nrTerms)=1;
sOPTIMstruct.cosalphaF(1:nrTerms)=1;
sOPTIMstruct.sinlB(1:nrTerms)=1;
sOPTIMstruct.coslB(1:nrTerms)=1;
sOPTIMstruct.sinalphaB(1:nrTerms)=1;
sOPTIMstruct.cosalphaB(1:nrTerms)=1;
sOPTIM = Struct2Vec(sOPTIMstruct,actParamNames);
%what values to be periodic
%almost every thing must be periodic
yPERIOD = ones(size(yINIT));
%except horizontal position, time, posWork
yPERIOD(contStateIndices.x) = 0;
yPERIOD(contStateIndices.time) = 0;
yPERIOD(contStateIndices.posWork) = 0;
%besides, the following requirements should be a side-product of
%periodicity in the continuous states,but can notify if something goes
%wrong
zPERIODstruct.phaseF = 1;
zPERIODstruct.phaseB = 1;
zPERIOD = Struct2Vec(zPERIODstruct,discStateNames);
%Compose the cost function
%No cost contribution from the continuous states
yCOSTstruct = struct();
yCOST = Struct2Vec(yCOSTstruct,contStateNames);
%Only the COT contributes to the cost function
zCOSTstruct.COT = 1;
zCOST = Struct2Vec(zCOSTstruct,discStateNames);
%No COST contribution from the system parameters
pCOSTstruct = struct();
pCOST = Struct2Vec(pCOSTstruct,sysParamNames)
%No cost contribution from from exciation parameters
sCOSTstruct = struct();
sCOST = Struct2Vec(sCOSTstruct,actParamNames)
%The yDependency helps speeding up  the computation of the Jacobian in the
%optimization by telling the optimizer which derivatives depends on which
%states
yDEPENDENCYstruct.x   = 0; %this state has no impact on the right hand side of the ODE->'0'
yDEPENDENCYstruct.dx = 1; %this state is the derivative of state #1 x->1
yDEPENDENCYstruct.y   = 0; %this state has no impact on the right hand side of the ODE->0
yDEPENDENCYstruct.dy = 3; %this state is the derivative of the state #3 y->3
yDEPENDENCYstruct.thetaB = -1; %All following states impact the right hand side ODE
yDEPENDENCYstruct.thetaF = -1;
yDEPENDENCYstruct.dthetaB = -1;
yDEPENDENCYstruct.dthetaF = -1;
yDEPENDENCYstruct.time = -1;
yDEPENDENCYstruct.posWork = 0;
dircolOptions.yDependency = Struct2Vec(yDEPENDENCYstruct,contStateNames);
%
%In the current case this is air-phase, back-stance, double-stance, front-stance,
% air-phase.  The corresponding event-sequence (event # in parenthesis)
% that is passed to the optimizer is thus: 'Back touch down (3)', 'Front
% touch down (1)', 'Back lift off (4)', 'Front lift off (2)', 'Stride time
% reached (5)':
dircolOptions.ordEvents = [3 1 4 2 5];
dircolOptions.nPerInterval = [15 15 15 15 15];
dircolOptions.numOPTS = optimset('MaxFunEvals',20000,'MaxIter',10000);
%Run the optimizer for a passive bounding
[yCYC,zCYC,pCYC,tEVENT_CYC,yGRID_CYC,costValue] =...
    FindPeriodicSolution_DIRCOL(@HybridDynamics, yINIT, zINIT, pINIT,...
    yOPTIM,zOPTIM,pOPTIM,...
    yPERIOD,zPERIOD,...
    yCOST,zCOST,pCOST,...
    dircolOptions);
%Store the results in a file for later use
%for windows
save('Data\OptimalBouding.mat','yCYC','zCYC','pCYC','tEVENT_CYC','yGRID_CYC',...
    'costValue',...
    'yINIT', 'zINIT', 'pINIT',...
    'yOPTIM','zOPTIM','pOPTIM',...
    'yPERIOD','zPERIOD',...
    'yCOST', 'zCOST','pCOST',...
    'dircolOptions');