function [sysParamVec, sysParamNames, sysParamIndices] = SysParamDefinition()
%All the parameters here are non-dimensionalized parameters.
% Parameters of the model

sysParam.IB = 0.3;   % Back body Inertia
sysParam.IF = 0.5;   % Front body Inertia
sysParam.kB = 18;      % Stiffness of Back knee
sysParam.kF =  18;      % Stiffness of Front knee
sysParam.kS =   5;     % stiffness of body spine
sysParam.lB = 0.1;   % Back body length from Back Body COM to Back Hip
sysParam.lF = 0.3;   % Front body length from Front Body COM to Front Hip
sysParam.lD = 0.2;   % lower segment length of the leg(from knee joint to foot)
sysParam.alpha = 60*pi/180; % [rad] attack angle
sysParam.Fr = 4; % Froude number
%sysParam.thetaB_0 = 0*pi/180; % [rad] resting Back Body angle
%sysParam.thetaF_0 = 0*pi/180; % [rad] resting Front Body angle
%sysParam.phiB_0 = 0*pi/180; % [rad] resting Back Hip angle

[sysParamVec, sysParamNames] = Struct2Vec(sysParam);
sysParamIndices = Vec2Struct(1:1:length(sysParamVec),sysParamNames);

end


