function [contStateVec, contStateNames, contStateIndices] = ContStateDefinition()
% define the continuous state vector for a bouding quadruped. all
% parameters here are nondimensional ones

    contState.x = 0.0; % horizontal position
    contState.dx = 0.7; % horizontal velocity
    contState.y = 1.15; % vertical position
    contState.dy = 0.0; % vertical velocity
    contState.thetaB = 0.5*pi/180; % BackBody angle
    contState.thetaF = 0.2*pi/180;% FrontBody angle
    contState.dthetaB = 5*pi/180; % BackBody angular velocity
    contState.dthetaF = -5*pi/180; % FrontBoday angular velocity
    contState.time = 0.0; % time that has passed since the start of the step
    contState.posWork = 0.0; % positive mechanical work of the actuators

    [contStateVec, contStateNames] = Struct2Vec(contState);
    contStateIndices = Vec2Struct(1:1:length(contStateVec),contStateNames);
    
end
