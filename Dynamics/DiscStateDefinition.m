function [discStateVec, discStateNames, discStateIndices] = DiscStateDefinition()
% all the parameters are non-dimensionalized ones.
    discState.phaseF = 1; % the current phase of the front leg (stance = 2, flight = 1,3)
    discState.phaseB = 1; % the current phase of the back leg (stance = 2, flight = 1,3)
    discState.COT = 0; % cost of transportation (posWork/distance traveled)
    discState.contPtB = 0; % contact point of Back foot or Back foot location
    discState.contPtF = 0; % contact point of Front foot or Front foot location
    
    [discStateVec, discStateNames] = Struct2Vec(discState);
    discStateIndices = Vec2Struct(1:1:length(discStateVec), discStateNames);
    
end