function [yPLUS, zPLUS, isTerminal] = JumpMap(yMINUS, ZMINUS, p, event)
% get a mapping for the state and parameter vectors. This allows us to use
% a more readable syntax:: "y(contStateIndices.dy)" instead of "y(3)"
persistent contStateIndices sysParamIndices discStateIndices
if isempty(contStateIndices) || isempty(sysParamIndices) || isempty(discStateIndices)
    [~,~,contStateIndices] = ContStateDefinition();
    [~,~,sysParamIndices] = SysParamDefinition();
    [~,~,discStateIndices] = DiscStateDefinition();
end

% at first, set as the incoming states
yPLUS = yMINUS;
zPLUS = zMINUS;

% Event 1: detect touchdown of back leg.
% Event 2: detect liftoff of back leg.
% Event 3: detect apex transit, dy == 0 during flight for back leg.
% Event 4: detect touchdown of the front leg
% Event 5: detect liftoff of front leg. 
% Event 6: detect apex transit, dy ==0 during flight for front leg.

switch event
    case 1 % Event 1: detect touchdown of back leg.
        % store the foot location when it touch the ground.
        %xB = xG-(lF/(lB+lF))*(lB*cos(thetaB)+lF*cos(thetaF));
        %xBH = xB - lB*cos(thetaB)
        %xBF = xBH + cos(alpha)
        MINUS_CoMB_horzlocation =  yMINUS(contStateIndices.x)-(p(sysParamIndices.lF)/(p(sysParamIndices.lB)+p(sysParamIndices.lF)))*(p(sysParamIndices.lB)*cos(yMINUS(contStateIndices.thetaB))+p(sysParamIndices.lF)*cos(yMINUS(contStateIndices.thetaF)));
        MINUS_CoMBH_horzlocation = MINUS_CoMB_horzlocation - p(sysParamIndices.lB)*cos(yMINUS(contStateIndices.thetaB));
        zPLUS(discStateIndices.contPtB) = MINUS_CoMBH_horzlocation + cos(p(sysParamIndices.alpha));
        
        % set a new phase
        zPLUS(discStateIndices.phaseB) = 2;
        %Intermediate event. Simulation continues
        isTerminal = false;
    case 2 % Event 2: detect liftoff of the back leg.
        %nothing
        %set a new phase
        zPLUS(discStateIndices.phaseB) = 3;
        %Intermidiate. Simulation continues
        isTerminal = false;
    case 3 % Event 3: detect apex transit, dy == 0 during flight for back leg. 
        %set a new phase
        zPLUS(discStateIndices.phaseB) = 1;
        % this event only stops the simulation
        isTerminal = false;
    case 4 % Event 4: detect touchdown of the front leg.
        %store the foot location when it touch the ground.
        %xF = xG+(lB/(lB+lF))*(lB*cos(thetaB)+lF*cos(thetaF));
        %xFH = xF + lF*cos(thetaF)
        %xFF = xFH + cos(alpha)
        MINUS_CoMF_horzlocation = yMINUS(contStateIndices.x)+(p(sysParamIndices.lB)/(p(sysParamIndices.lB)+p(sysParamIndices.lF)))*(p(sysParamIndices.lB)*cos(yMINUS(contStateIndices.thetaB))+p(sysParamIndices.lF)*cos(yMINUS(contStateIndices.thetaF)));
        MINUS_CoMFH_horzlocation = MINUS_CoMF_horzlocation + p(sysParamIndices.lF)*cos(yMINUS(contStateIndices.thetaF));
        zPLUS(discStateIndices.contPtF) = MINUS_CoMFH_horzlocation + cos(p(sysParamIndices.alpha));
        
        % set a new phase
        zPLUS(discStateIndices.phaseF) = 2;
        %Intermediate event. Simulation continues
        isTerminal = false;
    case 5 %Event 5: detect liftoff of the front leg
        % 
        % set a new phase
        zPLUS(discStateIndices.phaseF) = 3;
        %Simulation continues
        isTerminal = false;
    case 6 %Event 6: detect apex transit, dy == 0 during flight for front leg.
        zPLUS(discStateIndices.phaseF) = 1;
        isTerminal = true;
end
end

        
