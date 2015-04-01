function [actParamVec, actParamNames, actParamIndices] = actParamDefinition()
% All parameters are nondimensional ones. Parameters for actuation at spinal joint

actParam.strideFreq   = 0.5; % stride frequency
actParam.sinthetaS     = [ 0.12, 0.02,0,0,0,0,0,0,0,0]; % amplitude of the sine-terms for spine actuators... (check it back, follow Remy's idea for hip joint rotation)
actParam.costhetaS    = [-0.27,-0.03,0,0,0,0,0,0,0,0]; % amplitude of the cos-terms for spine actuators...(check it back)

[actParamVec, actParamNames] = Struct2Vec(actParam);
actParamIndices = Vec2Struct(1:1:length(actParamVec), actParamNames);

end
