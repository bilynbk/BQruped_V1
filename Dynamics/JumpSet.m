function eventVal = JumpSet(y, z, p)
% define the occurrence of discrete events that change the dynamics of the
% robot. Or this is conditions for determined when lift-off and touchdown
% events happen.
% Input:  - vector of continuous states 'y'
%            - vector of discrete states 'z'
%            - vector of model system parameters 'p'
% Output: - eventVal is returned. 
persistent contStateIndices actStateVec actStateIndices sysParamIndices discStateIndices
if isempty(contStateIndices) || isempty(actStateVec) || isempty(actStateIndices) || isempty(sysParamIndices) || isempty(discStateIndices)
    [~,                  ~, contStateIndices] = ContStateDefinition();
    [actStateVec, ~, actStateIndices]   = ActStateDefinition();
    [~                 , ~, sysParamIndices] = SysParamDefinition();
    [~                 , ~, discStateIndices] = DiscStateDefinition();
end

% Event 1: detect touchdown of back leg.
% Event 2: detect liftoff of back leg.
% Event 3: detect apex transit, dy == 0 during flight for back leg.
% Event 4: detect touchdown of the front leg
% Event 5: detect liftoff of front leg. 
% Event 6: detect apex transit, dy ==0 during flight for front leg.
n_events = 6;
eventVal = zeros(n_events, 1);

% calculate the extended parameters for using later on.
    %zB = zG-(lF/(lB+lF))*(lB*sin(thetaB)+lF*sin(thetaF));
    %zF = zG+(lB/(lB+lF))*(lB*sin(thetaB)+lF*sin(thetaF));
    % p(sysParamIndices.lF);p(sysParamIndices.lB);p(sysParamIndices.alpha);y(contStateIndices.thetaB);y(contStateIndices.thetaF);y(contStateIndices.dy)
    CoMB_height = y(contStateIndices.y)-(p(sysParamIndices.lF)/(p(sysParamIndices.lB)+p(sysParamIndices.lF)))*(p(sysParamIndices.lB)*sin(y(contStateIndices.thetaB))+p(sysParamIndices.lF)*sin(y(contStateIndices.thetaF))); 
    CoMF_height = y(contStateIndices.y)+(p(sysParamIndices.lB)/(p(sysParamIndices.lB)+p(sysParamIndices.lF)))*(p(sysParamIndices.lB)*sin(y(contStateIndices.thetaB))+p(sysParamIndices.lF)*sin(y(contStateIndices.thetaF)));
    
    %zBH = zB - lB*sin(thetaB)
    %zFH = zF + lF*sin(thetaF);
    hipB_height = CoMB_height - p(sysParamIndices.lB)*sin(y(contStateIndices.thetaB));
    hipF_height = CoMF_height + p(sysParamIndices.lF)*sin(y(contStateIndices.thetaF));

% Event 1: detect touchdown of back foot.
if z(discStateIndices.phaseB) == 1 % in flight
    %event is detected if foot is touch the ground, hip joint equals to leg length in vertical direction.
        
    ftB_height = hipB_height - sin(p(sysParamIndices.alpha)); % foot height of the back leg.
    
    % an event is defined by a transition from negative to positive. we
    % hence need a minus sign in the event function.
    eventVal(1) = -ftB_height;
else
    % only detect this event in flight phase
    eventVal(1) = -1;
end

%**************
%Event 2: detect liftoff of back foot
if z(discStateIndices.phaseB) == 2
    % event is detect if back leg is fully extended. compute the leg lenght
    %LLengthB = sqrt(cos(alpha)^2+zBH^2);
    legB_length = sqrt(hipB_height^2+cos(p(sysParamIndices.alpha))^2);
    
    % when this becomes larger than the uncompressed leg length (1), the event
    % is triggered.
    eventVal(2) = legB_length - 1;
else 
    % Only detect this event during stance.
    eventVal(2) = -1;
end

%***********
%Event 3: detect apex transit.
if z(discStateIndices.phaseB) == 3 % in flight and after passing the contact point.
    % Event is detected if the vertical velocity goes from positive to
    % negative. to detect a '-' to '+' transition, we need a minus sign in
    % the event function.    
    % p(sysParamIndices.lF);p(sysParamIndices.lB);p(sysParamIndices.alpha);y(contStateIndices.thetaB);y(contStateIndices.thetaF);y(contStateIndices.dy);y(contStateIndices.dthetaB);y(contStateIndices.dthetaF)
    % dyB: velocity of back body.
    % dyB = dzG - (dthetaF*lF^2*cos(thetaF))/(lB + lF) - (dthetaB*lB*lF*cos(thetaB))/(lB + lF)
    dyB = y(contStateIndices.dy) - (y(contStateIndices.dthetaF)*p(sysParamIndices.lF)^2*cos(y(contStateIndices.thetaF)))/(p(sysParamIndices.lB) + p(sysParamIndices.lF)) - (y(contStateIndices.dthetaB)*p(sysParamIndices.lB)*p(sysParamIndices.lF)*cos(y(contStateIndices.thetaB)))/(p(sysParamIndices.lB) + p(sysParamIndices.lF));
    
    eventVal(3) = -dyB;
else
    % only detect this event during flight after stance
    eventVal(3) = -1;
end

%***********
%Event 4: detect touchdown of front leg.
if z(discStateIndices.phaseF) = 1 % in flight
    %event is detected if foot is touch the ground, hip joint equals to leg length in vertical direction.
        
    ftF_height = hipF_height - sin(p(sysParamIndices.alpha)); % foot height of the back leg.
    
    % an event is defined by a transition from negative to positive. we
    % hence need a minus sign in the event function.
    eventVal(4) = -ftF_height;
else
    % only detect this event in flight phase.
    eventVal(4) = -1;
end
    
%**********
%Event 5: detect liftoff of front leg
if z(discStateIndices.phaseF) == 2
    % event is detect if front leg is fully extended. compute the leg lenght
    %LLengthF = sqrt(cos(alpha)^2+zFH^2);
    legF_length = sqrt(hipF_height^2+cos(p(sysParamIndices.alpha))^2);
    
    % when this becomes larger than the uncompressed leg length (1), the event
    % is triggered.
    eventVal(5) = legF_length - 1;
else 
    % Only detect this event during stance.
    eventVal(5) = -1;
end


%**********
%Event 6: detect apex transit of front leg.
if z(discStateIndices.phaseF) == 3 % in flight and after passing the contact point.
    % Event is detected if the vertical velocity goes from positive to
    % negative. to detect a '-' to '+' transition, we need a minus sign in
    % the event function.    
    % p(sysParamIndices.lF);p(sysParamIndices.lB);p(sysParamIndices.alpha);y(contStateIndices.thetaB);y(contStateIndices.thetaF);y(contStateIndices.dy);y(contStateIndices.dthetaB);y(contStateIndices.dthetaF)
    % dyF: velocity of front body.
    % dyF = dzG + (dthetaB*lB^2*cos(thetaB))/(lB + lF) + (dthetaF*lB*lF*cos(thetaF))/(lB + lF)
    dyF = y(contStateIndices.dy) + (y(contStateIndices.dthetaB)*p(sysParamIndices.lB)^2*cos(y(contStateIndices.thetaB)))/(p(sysParamIndices.lB) + p(sysParamIndices.lF)) + (y(contStateIndices.dthetaF)*p(sysParamIndices.lB)*p(sysParamIndices.lF)*cos(y(contStateIndices.thetaF)))/(p(sysParamIndices.lB) + p(sysParamIndices.lF));
    eventVal(6) = -dyF;
else
    % only detect this event during flight after stance
    eventVal(6) = -1;
end

end
