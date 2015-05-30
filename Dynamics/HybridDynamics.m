% *************************************************************************
%
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p)
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s)
%
% -> This MATLAB function simulates the hybrid dynamics associated with the
% functions 'JumpSet', 'JumpMap', and 'FlowMap'.  These functions must be
% available on the MATLAB search path:
%
% - eventValue = JumpSet(y,z,p,exctFcnHndl,s), describes a set of event
%            functions 'e', that trigger the corresponding event if a
%            zero-crossing is detected in positive direction.  
% - [yPLUS, zPLUS, isTerminal] = JumpMap(y ,z, p, exctFcnHndl, s, eventNr), 
%            describes a set of event handlers 'g' that define the
%            instantenous changes of the states during a specific event.
%            If the event is terminal (isTerminal == true), the
%            simulation will be aborted after this event. 
% - yDOT = FlowMap(y, z, p, exctFcnHndl, s), describes the continuous
%            dynamics 'f' of the hybrid system by a first order ODE.
%
% If the system is passive, the result of the simulation only depends on
% the initial continuous states 'yIN', the initial discrete states 'zIN',
% and a vector of parameters 'p'.  
%
% An active system depends additionally on an excitation function given by
% 'exctFcnHndl' with a vector of excitation parameters 's', according to:
%
% - u = ExctFcn(y, z, s), which returns the excitation input vector u for
%            every step of the simulation.
%
%
% HybridDynamics returns the continuous and discrete states after the first
% terminal event, together with the time when this happened.
%
% Additionally, the function can be provided with an output object
% 'outputIN' for graphical display or recording of the states, which must
% be derived from the class 'OutputCLASS'. It is updated throughout the
% simulation and returned as 'outputOUT':  
%
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, outputIN)
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, outputIN)
%
% Finally, additional options can be provided by an 'options' struct. The
% following options are provided: 
% - options.tIN  [0]   simulation will start at tIN
% - options.tMAX [inf] simulation will stop at tMAX (to indicate this has
%                      happened, tOUT will be set to -1).
% - options.odeOPTIONS this can be used to change the options for the
%                      MATLAB ODE solver. It must be created with the
%                      'odeset' function. The standard configuration sets: 
%                      'RelTol'=1e-6,'AbsTol'=1e-12,'MaxStep'=0.01.
%
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, options)
% function [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, options)
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, outputIN, options)
% function [yOUT, zOUT, tOUT, outputOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, outputIN, options)
%
%   See also FLOWMAP, JUMPMAP, JUMPSET, OUTPUTCLASS.
%
function [yOUT, zOUT, tOUT, varargout] = HybridDynamics(yIN, zIN, p, varargin)

    % *********************************************************************
    % INPUT HANDLING
    %   Check the number of parameters:
    if (nargin>7) || (nargin<3)
        error('GaitCreation:HybridDynamcis:WrongParameterCount', 'Wrong number of input arguments')
    end
    %   Check if we are dealing with an active system:
    if (nargin>3) && (isa(varargin{1},'function_handle'))
        exctFcnHndl = varargin{1};  
        s = varargin{2};
    else
        exctFcnHndl = [];
        s = [];
    end
    %   Check if an output object was provided:
    outputIN = [];
    if nargin>3
        for i = 1:nargin-3
            if isa(varargin{i},'OutputCLASS')
                outputIN = varargin{i};
            end
        end
    end
    %   Check if outputOUT is desired but no outputIN provided:
    if (nargout == 4) && isempty(outputIN)
        error('GaitCreation:HybridDynamcis:NoOutputObject', 'Output object can only be returned if an incoming output object is provided')
    end
    % Check if options were provided:
    options = struct([]);
    if (nargin>3) && (isa(varargin{end},'struct')) && ~(nargin == 7 && isa(varargin{3}, 'function_handle'))
        options = varargin{end};
    end
    % Evaluate options:
    if isfield(options,'tMAX')
        tMAX = options.tMAX;
    else
        tMAX = inf;
    end
    if isfield(options,'tIN')
        tIN = options.tIN;
    else
        tIN = 0;
    end
    % Define basic ode options, which can be overwritten by the user
    % provided values
    odeOPTIONS = odeset('RelTol',1e-6,...
                        'AbsTol',1e-12,...
                        'MaxStep',0.01);
    if isfield(options,'odeOPTIONS') % overwrite with the user provided values
        odeOPTIONS = odeset(odeOPTIONS, options.odeOPTIONS);
    end
    % Set up the options for output and event detection (do this last, as these should never be overwritten by the user):
    odeOPTIONS = odeset(odeOPTIONS,'Events',@Events,'OutputFcn',@OutputFcn);
    % END INPUT HANDLING
    % *********************************************************************
    
    
    
    % *********************************************************************
    % SIMULATE UNTIL TERMINAL EVENT
                     
    %   Start integration:
    isTerminal = false;
    
    %   Start clock for the timing of the output function
    if ~isempty(outputIN)
        tic
    end
    while ~isTerminal
        % Integrate until the next event, maximally for tMAX:
        if isempty(outputIN)
            tspan = [tIN,tMAX];
        else
            tspan = outputIN.getTimeVector(tIN,tMAX);
        end
        % NOTE: even though ode45 is provided with an initial column guess,
        % the results are stored in rows.
        [~,y,teOUT,yeOUT,ieOUT] = ode45(@ODE,tspan,yIN,odeOPTIONS);
        if isempty(ieOUT)        
            % No event occurred. The simulation ran out of time without
            % reaching the terminating event. Map final continuous states
            % (discrete states were not altered) and set time to -1:
            yIN = y(end,:)';  % This will be mapped to yOUT below.
            tIN = -1;
            break;
        else
            % Handle the discrete change of states at events by calling the
            % jump map (which must be on the MATLAB search path): 
            if isempty(exctFcnHndl)
                [yIN, zIN, isTerminal] = JumpMap(yeOUT(end,:)', zIN, p, ieOUT(end));
            else
                [yIN, zIN, isTerminal] = JumpMap(yeOUT(end,:)', zIN, p, exctFcnHndl, s, ieOUT(end));
            end
            tIN = teOUT(end);
            % Display the result of the discrete changes:
            OutputFcn(tIN,yIN,[]);
        end    
    end
    % Map states for return values
    yOUT = yIN;+
    zOUT = zIN;
    tOUT = tIN;
    if nargout == 4
        varargout(1) = {outputIN};
    else
        varargout = {};
    end
    % DONE SIMULATING UNTIL TERMINAL EVENT
    % *********************************************************************
    
    
    
    % *********************************************************************
    % Event Detection   
    function [value_,isterminal_,direction_] = Events(~,y_)
        % Get values of the event function by calling the jump set function
        % (which must be on the MATLAB search path):
        if isempty(exctFcnHndl)
            value_ = JumpSet(y_, zIN, p);
        else
            value_ = JumpSet(y_, zIN, p, exctFcnHndl, s);
        end
        n_events_ = length(value_);
        isterminal_ = ones(n_events_,1); % All events are terminal wrt the ode integration
        direction_  = ones(n_events_,1); % All events require a positive derivative
    end
    % End Event Detection
    % *********************************************************************
    
    % *********************************************************************
    % ODE of the continuous dynamics
    function dydt_ = ODE(~,y_)
        % Get continuous derivatives, by calling the flow map function
        % (which must be on the MATLAB search path):
        if isempty(exctFcnHndl)
            dydt_ = FlowMap(y_, zIN, p);
        else
            dydt_ = FlowMap(y_, zIN, p, exctFcnHndl, s);
        end
    end
    % End ODE
    % *********************************************************************
    
    % *********************************************************************
    % Call the updating function for the current state
    function status_ = OutputFcn(t_,y_,plot_flag_)
        if ~isempty(outputIN) 
            if isempty(plot_flag_)
                for j_ = 1:length(t_);
                    if isempty(exctFcnHndl)
                        u_ = [];
                    else
                        u_ = exctFcnHndl(y_(:,j_), zIN, s);
                    end
                    % call the update function as given in the update object
                    outputIN = update(outputIN, y_(:,j_), zIN, t_(j_), u_);
                    % Wait until actual time equals simulation time (times
                    % factor)
                    while toc<t_(j_)*outputIN.slowDown;
                    end
                end
            elseif strcmp(plot_flag_,'init') %First step:
                if isempty(exctFcnHndl)
                    u_ = [];
                else
                    u_ = exctFcnHndl(y_(:), zIN, s);
                end
                % call the update function as given in the update object
                outputIN = update(outputIN, y_(:), zIN, t_(1), u_);
            end
        end
        status_ = 0; % keep integrating
    end
    % End output
    % *********************************************************************
    
end
% *************************************************************************
% *************************************************************************