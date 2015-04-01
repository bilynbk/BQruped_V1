function [actStateVec, actStateNames, actStateIndices] = ActStateDefinition()

% all parameters are non-dimensionalized ones. for spinal joint actuator.
% check the correctness again...

actState.uthetaS  = 0; % motor angle of the spine rotational actuator (thetaS = thetaB - thetaF)
actState.duthetaS  = 0; % velocity ...

[actStateVec, actStateNames] = Struct2Vec(actState);
actStateIndices = Vec2Struct(1:1:length(actStateVec), actStateNames);

end



