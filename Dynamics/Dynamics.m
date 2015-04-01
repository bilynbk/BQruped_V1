function [yOUT,zOUT,tOUT,varargout] = Dynamics(yIN, zIN, p, varargin)

%***********************************************
%Input checking
% check the number of parameters:
    if (nargin>7) || (nargin<3)
        error('Wrong number of input arguments')
    end
    % check if the system is active
    if (nargin > 3) && (isa(varargin{1},'function_handle'))
        exctFcnHndl = varargin{1};
        s = varargin{2};
    else
        exctFcnHndl = [];
        s = [];
    end
    % check if an output object was provided
    outputIN = [];
    if nargin>3
        for i = 1:nargin-3
            if isa(varargin{i},'OutputCLASS')
                outputIN = varargin{i};
            end
        end
    end
    %check if outputOUT is desired but no outputIN provided                
    if (nargout == 4) && isempty(outputIN)
        error('Output object can only be returned if an incoming output object is provided')
    end
    % Check if options are provided
    options = struct([]);
    if (nargin >3) && (isa(varargin{end},'struct')) && ~(nargin == 7 && isa(varargin{3},'function_handle'))
        options = varargin{end};
    end
    %evaluate options
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
    %define basic ode options, 
    odeOPTIONS = odeset('RelTol',1e-8, ...
                                           'AbsTol',1e-12, ...
                                           'MaxStep', 0.01);
    if isfield(options,'odeOPTIONS') 
        odeOPTIONS = odeset(odeOPTIONS,options.odeOPTIONS);
    end
    % set up the options for output and event detection 
    odeOPTIONS = odeset(odeOPTIONS,'Events',@Events,'OutputFcn',@OutputFcn);
    %End input checking
    %****************************************
    
    %****************************************
    %simulate until terminal event
    % start integration
    isTerminal = false;
    
    % start clock for the timing of the ouput funtion
    if ~isempty(outputIN)
        tic
    end
    while ~isTerminal
        % integrate until the next event, maximally for tMAX:
        if isempty(outputIN)
            tspan = [tIN,tMAX];
        else
            tspan = outputIN.getTimeVector(tIN,tMAX);
        end
        %ODE solver ode45
        [~, y, teOUT, yeOUT, ieOUT] = ode45(@ODE,tspan,yIN,odeOPTIONS);
        if isempty(ieOUT)
            % no event occured. the simulation ran out of time without
            % reaching the terminating event. map final continuous states
            % (discrete states are not altered) and set time to -1
            yIN = y(end, :)'; % this will be mapped to yOUT below
            tIN = -1;
            break;
        else
            %handle the discrete change of states at events by calling the
            %jump map
            if isempty(exctFcnHndl)
                [yIN, zIN, isTerminal] = JumpMap(yeOUT(end,:)', zIN, p, ieOUT(end));
            else
                [yIN, zIN, isTerminal] = JumpMap(yeOUT(end,:)', zIN, p, exctFcnHndl, s, ieOUT(end));
            end
            tIN = teOUT(end);
            %display the result of the discrete changes
            OutputFcn(tIN, yIN, []);
        end
    end
    % Map states for return values
    yOUT = yIN;
    zOUT = zIN;
    tOUT  = tIN;
    if nargout == 4
        varargout(1) = {outputIN};
    else
        varargout = {};
    end
  
    % done simulating until terminal event
    %********************************************
    
    
    %******************************************
    %Event detection
    function [value_, isterminal_, direction_] = Events(~,y_)
        %Get values of the event function by calling the jump set function
        if isempty(exctFcnHndl)
            value_ = JumpSet(y_, zIN, p);
        else
            value_ = JumpSet(y_, zIN, p, exctFcnHndl, s);
        end
        n_events_ = length(value_);
        isterminal_ = ones(n_events_,1);
        direction_ = ones(n_events_,1);
    end
    %End Event detection
    %End Event Detection
    %*********************************************
    
    
    %*********************************************
    %ODE of the continuous dynamics
    function dydt_ = ODE(~,y_)
        %get the continuous derivatives, by calling the flow map function 
        if isempty(exctFcnHndl)
            dydt_ = FlowMap(y_, zIN, p);
        else
            dydt_ = FlowMap(y_, zIN, p, exctFcnHndl, s);
        end
    end
    %end ODE
    %*********************************************
    
    
    %*********************************************
    % Calling the updating function for the current state
    function status_ = OutputFcn(t_,y_,plot_flag_)
        if ~isempty(outputIN)
            if isempty(plot_flag_)
                for j_ = 1:length(t_)
                    if isempty(exctFcnHndl)
                        u_ = [];
                    else
                        u_ = exctFcnHndl(y_(:,j_), zIN, s);
                    end
                    % call the update function as given in the update object 
                    outputIN = update(outputIN, y_(:, j_), zIN, t_(j_), u_);
                    % wait until actual time equals simulation time
                    while toc < t_(j_)*outputIN.slowDown;
                    end
                end
            elseif strcmp(plot_flag_,'init') % first step:
                if isempty(exctFcnHndl)
                    u_ = [];
                else
                    u_ = exctFcnHndl(y_(:), zIN, s);
                end
                %call the update function as given in the update object
                outputIN = update(outputIN, y_(:), zIN, t_(1), u_);
            end
        end
    status_ = 0; % keep integrating
    end
    %End ouput
    %**************************************
    
end
%************************************************