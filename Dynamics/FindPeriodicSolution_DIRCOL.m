% *************************************************************************
%
% *************************************************************************
%
% function [yCYC, zCYC, pCYC, sCYC, tEVENT_CYC, yGRID_CYC, costValue] =  ...
%     FindPeriodicSolution_DIRCOL(hybrDynHndl, yINIT,   zINIT,  pINIT,  exctFcnHndl, sINIT, ... 
%                                              yOPTIM,  zOPTIM, pOPTIM,              sOPTIM, ... 
%                                              yPERIOD, zPERIOD, ...
%                                              yCOST,   zCOST,  pCOST,               sCOST, ...
%                                              options)    
%
% function [yCYC, zCYC, pCYC, tEVENT_CYC, yGRID_CYC, costValue] =  ...
%     FindPeriodicSolution_DIRCOL(hybrDynHndl, yINIT,   zINIT,  pINIT,... 
%                                              yOPTIM,  zOPTIM, pOPTIM,... 
%                                              yPERIOD, zPERIOD, ...
%                                              yCOST,   zCOST,  pCOST, ...
%                                              options)
%
% function [yCYC, zCYC, pCYC, sCYC, tEVENT_CYC, yGRID_CYC] =  ...
%     FindPeriodicSolution_DIRCOL(hybrDynHndl, yINIT,   zINIT,  pINIT,  exctFcnHndl, sINIT, ...
%                                              yOPTIM,  zOPTIM, pOPTIM,              sOPTIM,...
%                                              yPERIOD, zPERIOD, ...
%                                              options)
%
% function [yCYC, zCYC, pCYC, tEVENT_CYC, yGRID_CYC] =  ...
%     FindPeriodicSolution_DIRCOL(hybrDynHndl, yINIT,   zINIT,  pINIT,... 
%                                              yOPTIM,  zOPTIM, pOPTIM,... 
%                                              yPERIOD, zPERIOD, ...
%                                              options)
%
% This function returns a periodic solution for the model described by
% 'hybrDynHndl'. It starts a constrained optimization algorithm that
% minimizes a cost function composed by the sum of the final values of
% designated states and parameters while maintaining a peridic solution 
% in other states.  
% The states and parameters provided by the calling function ('yINIT',
% 'zINIT, ...) are either left unchanged, or serve as the initial guess
% for optimization.   
% Which states and parameters are optimized, must be periodic, or
% contribute to the cost function can be set via the flags '*OPTIM', 
% '*PERIOD', '*COST', ...
% Variables that are not periodic within a step, do not contribute to the
% cost function, and should have a fixed starting value (like the
% horizontal position) should be excluded.   
%
% Input:  - A one stride gait model with the following interface:
%           [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, exctFcnHndl, s, options)
%           [yOUT, zOUT, tOUT] = HybridDynamics(yIN, zIN, p, options)
%           This function maps a set of initial states to a set of final
%           states, using the given parameters. The user must support the
%           function handle 'hybrDynHndl' to this model.
%         - An initial guess for the continuous states 'yINIT'
%         - An initial guess for the discrete states 'zINIT'
%         - An initial guess for the system parameters 'pINIT'
%         OPTIONAL:
%           - An excitation function u = ExcitationFunction(y, z, s) that 
%             describes the position of the drive side of the series 
%             elastic actuator. If it is not provided the initial guess
%             from the definition function (EXCTSTATEDEFINITION) is used.
%           - A vector of excitation parameters 'sINIT';
%         - For each of these initial guess, three (two) sets of flags that
%           indicate:  
%           ... *OPTIM: The corresponding initial guess is being
%                       optimized [flag = 1] or not [flag = 0].
%           ... *PERIOD: The corresponding state is made periodic [flag = 1]
%                        or not [flag = 0].
%           OPTIONAL
%           ... *COST: The corresponding state or parameter contributes to
%                      the cost function [flag>0] in this case, the flag
%                      also indicates a weight that is used when the
%                      individual cost-terms are composed. If a
%                      state/parameter should not contribute to the cost
%                      function, the corresponding flag should be set to 0.
%         - Additional options are provided with the 'options' struct: 
%           * options.ordEvents A vector of all events that are encountered
%                               in this order during the simulation
%                               'ordEvents' (this MUST be provided) 
%           * options.nPerInterval A vector that indicates the number of
%                                  grid-points used in every single
%                                  integration interval. (this MUST be
%                                  provided)
%           * options.yGRID An initial grid for the continuous states.
%           * options.tEVENT A vector of points in time at which the events
%                            happen. This must be provided, if the grid is
%                            supplied by the user. 
%           * options.numOPTS A set of parameters for the numerical
%                             optimization(created by the 'optimset'
%                             function).  This will override the standard
%                             values set below.  It can, for example, be
%                             used to specify a number for maximal function
%                             evaluations. 
%           * options.yDependency A vector of length yINIT, that specifies,
%                                 if this state appears on the right and
%                                 side of the ODE.  Specifying this speeds
%                                 up the computation of the Jacobians.
%                                 Three possible values can be used:
%                                 a) '0' This state does not appear on the
%                                    right hand side of the ODE
%                                 b) 'n' This state is the direct
%                                    derivative of state n, but does not
%                                    appear anywhere else on the right hand
%                                    side of the ODE.
%                                 c) '-1' This state appears on the right
%                                    and side of the ODE. This is the
%                                    standard value.
%           * options.varJac [1e-5] Defines the size of the numerical
%                            differences that are used to compute the
%                            Jacobians in the optimization 
%                                 
%
% Output: - Initial continuous states 'yCYC', that result in an optimal
%           periodic motion 
%         - Initial discrete states 'zCYC', that result in an optimal
%           periodic motion
%         - System parameters 'pCYC', that result in an optimal
%           periodic motion
%         - The complete grid of support points 'yGRID_CYC'
%         - The points in time at which the events happened 'tEVENT_CYC'
%         OPTIONAL:
%           - Excitation parameters 'sCYC', that result in an optimal
%             periodic motion
%           - The final value of the overall cost-function 'costValue'
<<<<<<< HEAD
%
% Created by C. David Remy on 03/14/2011
%
=======
>>>>>>> origin/master
%   See also HYBRIDDYNAMICS, FLOQUETANALYSIS.
%
function [yCYC, zCYC, pCYC, varargout] = FindPeriodicSolution_DIRCOL(hybrDynHndl, yINIT,   zINIT,  pINIT, varargin)
    
    % *********************************************************************
    % Input handling
    nDIM = size(yINIT,1);
    if ((nargin ~= 10) && (nargin ~= 13) && (nargin ~= 17))
        error('GaitCreation:FindPeriodicSolution_DIRCOL:wrongParameterCount', 'Wrong number of input arguments.  Should be 10 for a passive system with no cost function, 13 for a passive system with cost function or an active system with no cost function, and 17 for an active system with cost function')
    end
    % Create some detailed information for the screen:
    disp('Looking for a periodic gait...');
    if isa(varargin{1},'function_handle')
        disp(' ... for an active system');
        activeSyst  = true;
        exctFcnHndl = varargin{01};
        sINIT       = varargin{02};
        yOPTIM      = varargin{03};
        zOPTIM      = varargin{04};
        pOPTIM      = varargin{05};
        sOPTIM      = varargin{06};
        yPERIOD     = varargin{07};
        zPERIOD     = varargin{08};
    else
        disp(' ... for a passive system');
        activeSyst  = false;
        yOPTIM      = varargin{01};
        zOPTIM      = varargin{02};
        pOPTIM      = varargin{03};
        yPERIOD     = varargin{04};
        zPERIOD     = varargin{05};
        sINIT       = [];
        sOPTIM      = [];
    end
    if (nargin == 17 && activeSyst)  || (nargin == 13 && ~activeSyst) 
        disp(' ... with a cost function');
        costFct = true;
        if activeSyst % active system with cost function
            yCOST       = varargin{09};
            zCOST       = varargin{10};
            pCOST       = varargin{11};
            sCOST       = varargin{12};
            options     = varargin{13}; 
        else
            yCOST       = varargin{06};
            zCOST       = varargin{07};
            pCOST       = varargin{08};
            sCOST       = [];
            options     = varargin{09}; 
        end
    else
        disp(' ... without a cost function');
        costFct = false;
        if activeSyst % active system without cost function
            options  = varargin{09}; 
        else
            options  = varargin{06};
        end
    end
    
	% Evaluate options:
    if isfield(options,'ordEvents')
        ordEvents = options.ordEvents;
        nEvents   = size(ordEvents, 2); 
    else
        error('GaitCreation:FindPeriodicSolution_DIRCOL:noOrdEvents', 'You need to provide an option ordEvents, which determines which events are encountered during the simulation in which order');
    end
    if isfield(options,'nPerInterval')
        nPerInterval = options.nPerInterval;
    else
        error('GaitCreation:FindPeriodicSolution_DIRCOL:noNPerInterval', 'You need to provide an option nPerInterval, how many grid points are used per integration interval');
    end
    if isfield(options,'yGRID')
        if isfield(options,'nPerInterval')
            yGRID = options.yGRID;
            tEVENT = options.tEVENT;
            disp('Using provided initial Grid...')
        else
            error('GaitCreation:FindPeriodicSolution_DIRCOL:noTEVENT', 'You need to provide an option tEVENT, if you provide the option yGRID');
        end  
        nGRIDnoMatch = false;
        for i = 1:nEvents
            if nPerInterval(i) ~= size(yGRID{i},2)
                options.nPerInterval(i) = size(options.yGRID{i},2);
                nGRIDnoMatch = true;
            end
        end
        if nGRIDnoMatch
            warning('GaitCreation:FindPeriodicSolution_DIRCOL:nPerIntervalMissmatch','options.nPerInterval and options.nGRID did not match in segment size.  Optimizer will use the values of options.nGRID');
        end
    else
        yGRID  = [];
        tEVENT = [];
    end
    if isfield(options,'yDependency')
        % Use the ODE right hand side dependencies supplied by the user:
        yDependency = options.yDependency;
    else
        % if none were supplied, assume that all states are dependent.
        yDependency = -ones(size(yINIT));
    end
    if isfield(options,'varJac')
        % The numerical disturbance to compute the Jacobians is supplied by
        % the user
        varJac = options.varJac;
    else
        % Use standard value:
        varJac = 1e-5;
    end
    % Define basic options for the numerical optimization/root search,
    % which can be overwritten by the user:
    if costFct % Systems with a cost function
        numOPTS = optimset('Algorithm','active-set',...
                           'Display','iter',...
                           'MaxFunEvals',2000,...
                           'MaxIter',1000,...
                           'TolX',1e-8,...
                           'TolFun',1e-8,...
                           'TolCon',1e-6,...
                           'LargeScale','off',...
                           'DiffMinChange',1e-12,...
                           'DiffMaxChange',1e-3,...
                           'RelLineSrchBnd',1e-3,...
                           'RelLineSrchBndDuration',1e8,...
                           'GradConstr','on',...
                           'GradObj','on');
    else % Systems without a cost function
        numOPTS = optimset('Algorithm','levenberg-marquardt',... 
                           'Display','iter',...
                           'MaxFunEvals',1000,...
                           'MaxIter',500,...
                           'TolFun',1e-10,...
                           'TolX',1e-10,...
                           'Jacobian','on');
    end
    if isfield(options,'numOPTS') % overwrite with the user provided values
        numOPTS = optimset(numOPTS, options.numOPTS);
    end
    
    % Create more screen output
    if ~isempty(yINIT(yPERIOD == 1))
        disp('  Making the following continuous DOFs periodic:');
        [~,contStateNames] = ContStateDefinition();
        disp(contStateNames(yPERIOD == 1));
    end
    if ~isempty(zINIT(zPERIOD == 1))
        disp('  Making the following discrete DOFs periodic:');
        [~,discStateNames] = DiscStateDefinition();
        disp(discStateNames(zPERIOD == 1));
    end
    if ~isempty(yINIT(yOPTIM == 1))
        disp('  Optimizing the following continuous DOFs:');
        [~,contStateNames] = ContStateDefinition();
        disp(contStateNames(yOPTIM == 1));
    end
    if ~isempty(zINIT(zOPTIM == 1))
        disp('  Optimizing the following discrete DOFs:');
        [~,discStateNames] = DiscStateDefinition();
        disp(discStateNames(zOPTIM == 1));
    end
    if ~isempty(pINIT(pOPTIM == 1))
        disp('  Optimizing the following system parameters:');
        [~,exctParamNames] = SystParamDefinition();
        disp(exctParamNames(pOPTIM == 1));
    end
    if activeSyst && ~isempty(sINIT(sOPTIM == 1))
        disp('  Optimizing the following controller parameters:');
        [~,exctParamNames] = ExctParamDefinition();
        disp(exctParamNames(sOPTIM == 1));
    end
    if costFct && ~isempty(yINIT(yCOST == 1))
        disp('  The following continuous DOFs are contributing to the cost function:');
        [~,contStateNames] = ContStateDefinition();
        disp(contStateNames(yCOST == 1));
    end
    if costFct && ~isempty(zINIT(zCOST == 1))
        disp('  The following discrete DOFs are contributing to the cost function:');
        [~,discStateNames] = DiscStateDefinition();
        disp(discStateNames(zCOST == 1));
    end
    if costFct && ~isempty(pINIT(pCOST == 1))
        disp('  The following system parameters are contributing to the cost function:');
        [~,exctParamNames] = SystParamDefinition();
        disp(exctParamNames(pCOST == 1));
    end
    if costFct && activeSyst && ~isempty(sINIT(sCOST == 1))
        disp('  The following control parameters are contributing to the cost function:');
        [~,exctParamNames] = ExctParamDefinition();
        disp(exctParamNames(sCOST == 1));
    end
    % END INPUT HANDLING
    % *********************************************************************
    
    
    % *********************************************************************
    % Create an initial grid, if it was not provided by the user:
    % Use the grid provided via DirColOpts:
    if isempty(yGRID)
        % Get a first initial guess from the *INIT values:
        disp('Creating initial Grid...')
        recOUTPUT = RecordStateCLASS();
        if activeSyst
            [~, ~, ~, recOUTPUT] = hybrDynHndl(yINIT, zINIT, pINIT, exctFcnHndl, sINIT, recOUTPUT);
        else
            [~, ~, ~, recOUTPUT] = hybrDynHndl(yINIT, zINIT, pINIT, recOUTPUT);
        end
        simRES = recOUTPUT.retrieve();
        % Pad the solution with the values at t=0:
        simRES.t = [0,simRES.t]';
        simRES.continuousStates = [yINIT,simRES.continuousStates];
        simRES.discreteStates   = [zINIT,simRES.discreteStates];
        % Events are detected by changes in the discrete states
        tEVENT = simRES.t(any(diff(simRES.discreteStates,1,2)~=0,1));
        % A special case is the terminal event, which doesn't necessarily
        % change the discrete states, but it should always exist.  So in
        % doubt, we add it 'manually':
        if tEVENT(end) ~= simRES.t(end)
            tEVENT = [tEVENT;simRES.t(end)];
        end
        if length(tEVENT) ~= nEvents
            error('GaitCreation:FindPeriodicSolution_DIRCOL:wrongEventCount', 'Wrong number of detected events.  When creating an initial guess, the function detected a different number of events than provided by the user')
        end
        % Create the initial grid over the continuous states:
        yGRID = cell(1,nEvents);
        tBORDER = [0;tEVENT];
        for i = 1:nEvents
            t = linspace(tBORDER(i)+1e-12, tBORDER(i+1)-1e-12, nPerInterval(i)+1);
            yGRID{i} = interp1(simRES.t' + linspace(0,1e-12,size(simRES.t,1)), simRES.continuousStates', t(2:end))';
        end
    end
    % *********************************************************************
    
    
    
        
    % *********************************************************************
    % Create vector of an combined initial guess:
    xINIT = MapYZPStoX(yINIT, zINIT, pINIT, sINIT, tEVENT, yGRID);

    % Apply the appropriate algorithm:
    if ~costFct
        % Only finding a periodic solution:
        disp('Running fsolve...');
        [xCYC, constViol] = fsolve(@RootTransferFunction, xINIT, numOPTS);
        [yCYC, zCYC, pCYC, sCYC, tEVENT_CYC, yGRID_CYC ] = MapXtoYZPS(xCYC);
        if activeSyst
            varargout = {sCYC, tEVENT_CYC, yGRID_CYC,(constViol'*constViol)};
        else
            varargout = {tEVENT_CYC, yGRID_CYC,(constViol'*constViol)};
        end
        disp([' Residual = ', num2str(constViol'*constViol)]);
        disp('...done with root search'); 
    else
        % Minimization of cost-function:
        disp('Running fmincon...');
        % Create additional linear constraints, ensuring that every
        % integration interval spans a positive time interval (of at least
        % 1e-5).  This prohibits negative time in the individual intervals.
        b = -ones(nEvents,1)*1e-5;
        A = zeros(nEvents, size(xINIT,1));
        for i = 1:nEvents
            if i>1
                A(i,nnz(yOPTIM) + nnz(zOPTIM) + nnz(pOPTIM) + nnz(sOPTIM) + i - 1) = 1;
            end
            A(i,nnz(yOPTIM) + nnz(zOPTIM) + nnz(pOPTIM) + nnz(sOPTIM) + i) = -1;
        end
        
        [xCYC, costValue, ~, output] = fmincon(@CostFunction, xINIT, A, b, [], [], [], [], @ConstrainFunction, numOPTS);
        [yCYC, zCYC, pCYC, sCYC, tEVENT_CYC, yGRID_CYC ] = MapXtoYZPS(xCYC);
        if activeSyst
            varargout = {sCYC, tEVENT_CYC, yGRID_CYC, costValue,output.constrviolation};
        else
            varargout = {tEVENT_CYC, yGRID_CYC, costValue,output.constrviolation};
        end
        disp([' Residual  = ', num2str(output.constrviolation)]);
        disp([' Costvalue = ', num2str(costValue)]);
        disp('...done with optimization'); 
    end
    % *********************************************************************
    
    
    % *********************************************************************
    % Local functions:
    function [y_, z_, p_, s_, tEVENT_, yGRID_] = MapXtoYZPS(x_)
        % In this function, the optimization vector x_ is mapped into
        % individual states and parameters. y_, z_, p_, s_, and tEVENT_ are
        % one-dimensional column vectors. yGRID is a cell arry.

        % Copy the initial values, as most of them remain unchanged.
        y_ = yINIT;
        z_ = zINIT;
        p_ = pINIT;
        s_ = sINIT;
        % Change the ones that were optimized:
        startIndex_ = 0;
        y_(yOPTIM == 1) = x_(startIndex_+1:startIndex_+nnz(yOPTIM == 1));
        startIndex_ = startIndex_ + nnz(yOPTIM == 1);
        z_(zOPTIM == 1) = x_(startIndex_+1:startIndex_+nnz(zOPTIM == 1));
        startIndex_ = startIndex_ + nnz(zOPTIM == 1);
        p_(pOPTIM == 1) = x_(startIndex_+1:startIndex_+nnz(pOPTIM == 1));
        startIndex_ = startIndex_ + nnz(pOPTIM == 1);
        s_(sOPTIM == 1) = x_(startIndex_+1:startIndex_+nnz(sOPTIM == 1));
        startIndex_ = startIndex_ + nnz(sOPTIM == 1);
        tEVENT_ = x_(startIndex_+1:startIndex_+nEvents);
        startIndex_ = startIndex_ + nEvents;
        % Extract yGRID for each integration interval
        yGRIDReshaped_ = x_(startIndex_+1:end);
        yGRID_ = cell(1,nEvents);
        for i_ = 1:nEvents
            yGRID_{i_} = reshape(yGRIDReshaped_(1:nDIM*nPerInterval(i_)),nDIM,nPerInterval(i_));
            yGRIDReshaped_ = yGRIDReshaped_(nDIM*nPerInterval(i_)+1:end);
        end
        if tEVENT_(1)<=0
            tEVENT_(1) = 0 +1e-12;
        end
        for i_ = 2:nEvents
            if tEVENT_(i_)<=tEVENT_(i_-1);
                tEVENT_(i_) = tEVENT_(i_-1) + 1e-12;
            end
        end
        % Make sure, time is monotonically increasing from event to event
        assert(tEVENT_(1)>0);
        for i_ = 2:nEvents
            assert(tEVENT_(i_)>tEVENT_(i_-1));
        end
    end


    function x_ = MapYZPStoX(y_, z_, p_, s_, tEVENT_, yGRID_)
        % Compose the optimization vector from the individual components
        % that are being optimized:
        % The optimization vector x consists of the following variables:
        % - yINIT(yOPTIM==1)
        % - zINIT(zOPTIM==1)
        % - pINIT(pOPTIM==1)
        % - sINIT(sOPTIM==1)
        % - tEVENT
        % - yGRID, support points for each segment (without the first elements,
        % which are directly computed from the Events/Initial-conditions). They
        % are given in the order: dimensions, segments, integration intervals:   
        yGRIDReshaped_ = [];
        for i_ = 1:nEvents
            yGRIDReshaped_ = [yGRIDReshaped_;reshape(yGRID_{i_},nDIM*nPerInterval(i_),1)];
        end
        x_ = [y_(yOPTIM == 1);
              z_(zOPTIM == 1);
              p_(pOPTIM == 1);
              s_(sOPTIM == 1);
              tEVENT_;
              yGRIDReshaped_];
    end
    

    function [c_, ceq_, Jc_, Jceq_] = ConstrainFunction(x_)
        %% The constraint function just calls the RootTransferFunction, as
        % inequality constraints are not required:
        c_ = [];
        Jc_ = [];
        [ceq_, Jceq_] = RootTransferFunction(x_);
        Jceq_ = Jceq_';
    end


    function [f_, Jf_] = CostFunction(x_)
        % The cost function simply evaluates the last segment:
        [~, zSTART_, p_, s_, ~, yGRID_] = MapXtoYZPS(x_);
        % Compute the final value of zOUT_:
        z_ = zeros(size(zINIT,1),nEvents);
        for i_ = 1:nEvents
            if i_ == 1
                z_(:,i_) = zSTART_;
            else
                % The evaluate the JumpMap on the final states of the
                % previous interval:
                if ~activeSyst
                    [~, zPLUS_] = JumpMap(yGRID_{i_-1}(:,end), z_(:,i_-1), p_, ordEvents(i_-1));
                else
                    [~, zPLUS_] = JumpMap(yGRID_{i_-1}(:,end), z_(:,i_-1), p_, exctFcnHndl, s_, ordEvents(i_-1));
                end
                z_(:,i_) = zPLUS_;
            end
        end
        % The terminal event:
        if ~activeSyst
            [yOUT_, zOUT_] = JumpMap(yGRID_{end}(:,end), z_(:,end), p_, ordEvents(end));
        else
            [yOUT_, zOUT_] = JumpMap(yGRID_{end}(:,end), z_(:,end), p_, exctFcnHndl, s_, ordEvents(end));
        end
        f_ = sum(yOUT_.*yCOST) + sum(zOUT_.*zCOST) + sum(p_.*pCOST) + sum(s_.*sCOST);
        
        
        if nargout>1  % Gradient is required
            Jf_ = zeros(1,size(x_,1));
            var_ = varJac;
            % Compute gradient:
            % Influence onto the cost function due to change in...
            % a) ySTART_
            % b) zSTART_
            % c) p_
            % d) s_
            % ... is computed via numerical differentiation on the cost
            % function
            for i_  = 1 :  nnz(yOPTIM) + nnz(zOPTIM) + nnz(pOPTIM) + nnz(sOPTIM)
                x_var_ = zeros(size(x_));
                x_var_(i_) = var_;
                Jf_(i_) = (CostFunction(x_ + x_var_)-f_)/var_;
            end
            % e) yGRID_
            % ... is only computed for the last segment in each integration
            % interval.  All other grid points have no impact on the cost
            % function.
            for i_ = 1:nEvents
                for k_ = 1:nDIM
                    j_ = OptimizationVariableIndex(i_, nPerInterval(i_)+1, k_);
                    x_var_ = zeros(size(x_));
                    x_var_(j_) = var_;
                    Jf_(j_) = (CostFunction(x_ + x_var_)-f_)/var_;
                end
            end
        end
    end


    function [ceq_, Jceq_] = RootTransferFunction(x_)
        % Convert optimization vector 'x_' into the different variables
        % used in the direct collocation: 
        [ySTART_, zSTART_, p_, s_, tEVENT_, yGRID_] = MapXtoYZPS(x_);
        % The constraint vector ceq_ consists of the following variables:
        % - Segment constraint violations (in the order dimension,
        %   segments, integration intervals)  [nDIM*sum(nPerInterval)
        %   elements. 
        % - Periodicity in y [nnz(yPERIOD) elements]
        % - Periodicity in z [nnz(zPERIOD) elements]
        % - Event constraint violation [nEvents elements]
        ceq_ = zeros(sum(nPerInterval)*nDIM+nnz(yPERIOD)+nnz(zPERIOD)+nEvents,1);
        % reserve some space for the Jacobian (if required):
        if nargout>1
            Jceq_ = zeros(sum(nPerInterval)*nDIM+nnz(yPERIOD)+nnz(zPERIOD)+nEvents,size(x_,1));
            % define the size of the disturbance used for finite differences:
            var_ = varJac;
        end
        % The grid of each integration interval is expanded with  a first
        % element (which depends on the initial conditions for the first
        % segment and the outcome of the jump-map for the subsequent
        % segments). This ensures periodicity over the borders of the
        % integration intervals.  The only additional constraints, that we
        % have to add are the overall periodicity and the event function at
        % the end of each integration interval.  The discrete states are
        % computed alongside and stored in the vector 'z_':
        z_ = zeros(size(zINIT,1),nEvents);
        % For every integration interval:
        for i_ = 1:nEvents
            if i_ == 1
                % For the first integration interval, use the initial values
                % provided by the optimizer:
                y_ = [ySTART_,yGRID_{i_}];
                z_(:,i_) = zSTART_;
            else
                % The evaluate the JumpMap on the final states of the
                % previous interval:
                if ~activeSyst
                    [yPLUS_, zPLUS_] = JumpMap(y_(:,end), z_(:,i_-1), p_, ordEvents(i_-1));
                else
                    [yPLUS_, zPLUS_] = JumpMap(y_(:,end), z_(:,i_-1), p_, exctFcnHndl, s_, ordEvents(i_-1));
                end
                y_ = [yPLUS_,yGRID_{i_}];
                z_(:,i_) = zPLUS_;
            end
            % Create a vector of evenly spaced times for the current
            % integration interval: 
            if i_ == 1
            	t_  = linspace(0, tEVENT_(i_), nPerInterval(i_) + 1);
            else
                t_  = linspace(tEVENT_(i_-1), tEVENT_(i_), nPerInterval(i_) + 1);
            end
            % The length of each segment is constant:
            h_ = t_(2)-t_(1);
            % Pre-compute all f(x) at the support points:
            f_ = zeros(nDIM, size(y_,2));
            for j_ = 1:size(y_,2)
                if ~activeSyst
                    f_(:,j_) = FlowMap(y_(:,j_), z_(:,i_), p_);
                else
                    f_(:,j_) = FlowMap(y_(:,j_), z_(:,i_), p_, exctFcnHndl, s_);
                end
            end
            % Compute the constraint violation for every segment:
            for j_ = 1:nPerInterval(i_)
                derViol_ = ComputeDerivativeViolation(y_(:,j_), y_(:,j_+1), f_(:,j_), f_(:,j_+1), z_(:,i_), p_, s_, h_);
                % And store it in the overall constraint vector:
                index_ = DerivativeConstraintIndex(i_, j_);
                ceq_(index_+1 : index_+nDIM) = derViol_;
            end
            % When the gradients are required, we compute partial Jacobians
            % here.  They have to be arranged in a complete Jacobian later:
            if nargout>1
                % do one state at the time:
                for k_ = 1:nDIM % for all states
                    % Check if this state actually influences the other
                    % states.  If yes, do a regular Jacobian computation:
                    if yDependency(k_) == -1
                        % Create a disturbance vector in y_:
                        y_var_     = zeros(nDIM,1);
                        y_var_(k_) = var_;
                        % Compute all f(x) at the disturbed control points
                        % (i.e., all but the first):
                        f_PLUS_ = zeros(nDIM, size(y_,2)-1);
                        for j_ = 2:size(y_,2)
                            if ~activeSyst
                                f_PLUS_(:,j_) = FlowMap(y_(:,j_) + y_var_, z_(:,i_), p_);
                            else
                                f_PLUS_(:,j_) = FlowMap(y_(:,j_) + y_var_, z_(:,i_), p_, exctFcnHndl, s_);
                            end
                        end
                        % Compute the changes to the constraint violations
                        % caused by these control points: 
                        for j_ = 2:size(y_,2)
                            % Alterations to this grid point do affect the
                            % segment to the left:
                            derViol_PLUS_ = ComputeDerivativeViolation(y_(:,j_-1), y_(:,j_) + y_var_, f_(:,j_-1), f_PLUS_(:,j_), z_(:,i_), p_, s_, h_);
                            indexC_ = DerivativeConstraintIndex(i_, j_-1);
                            indexX_ = OptimizationVariableIndex(i_, j_, k_);
                            Jceq_(indexC_+1 : indexC_+nDIM, indexX_) = (derViol_PLUS_ - ceq_(indexC_+1 : indexC_+nDIM))/var_;
                            % Alterations to this grid point do affect the
                            % segment to the right unless this is the last segment: 
                            if j_ < size(y_,2)
                                derViol_PLUS_ = ComputeDerivativeViolation(y_(:,j_)+ y_var_, y_(:,j_+1) , f_PLUS_(:,j_), f_(:,j_+1), z_(:,i_), p_, s_, h_);
                                indexC_ = DerivativeConstraintIndex(i_, j_);
                                indexX_ = OptimizationVariableIndex(i_, j_, k_);
                                Jceq_(indexC_+1 : indexC_+nDIM, indexX_) = (derViol_PLUS_ - ceq_(indexC_+1 : indexC_+nDIM))/var_;
                            end
                        end
                    else
                        % if not, set to entries for
                        for j_ = 2:size(y_,2)
                            indexC_ = DerivativeConstraintIndex(i_, j_-1);
                            indexX_ = OptimizationVariableIndex(i_, j_, k_);
                            if yDependency(k_)
                                Jceq_(indexC_+yDependency(k_), indexX_) = 0.75;
                            end
                            Jceq_(indexC_+k_, indexX_) = -1.5/h_;
                            if j_ < size(y_,2)
                                indexC_ = DerivativeConstraintIndex(i_, j_);
                                indexX_ = OptimizationVariableIndex(i_, j_, k_);
                                if yDependency(k_)
                                    Jceq_(indexC_+yDependency(k_), indexX_) = 0.75;
                                end
                                Jceq_(indexC_+k_, indexX_) = 1.5/h_;
                            end
                        end
                    end
                end
            end % if nargout>1
        end % end 'for all integration intervals'
        
        % Compute the states at the very end of the integration:
        if ~activeSyst
            [yFINAL_, zFINAL_] = JumpMap(yGRID_{end}(:,end), z_(:,end), p_, ordEvents(end));
        else
            [yFINAL_, zFINAL_] = JumpMap(yGRID_{end}(:,end), z_(:,end), p_, exctFcnHndl, s_, ordEvents(end));
        end
        % Periodicity as required by the user (y_ an i_ are representing
        % the final segment at this point): 
        perViol_ = [yFINAL_(yPERIOD == 1) - ySTART_(yPERIOD == 1);... 
                    zFINAL_(zPERIOD == 1) - zSTART_(zPERIOD == 1)];
        ceq_(sum(nPerInterval)*nDIM + 1  :  sum(nPerInterval)*nDIM + nnz(yPERIOD) + nnz(zPERIOD)) = perViol_;
                
        % Event search constraints    
        for i_ = 1:nEvents
            % evaluate the event function at the end of each segment and
            % use the event value (which should be zero) as additional
            % constraint
            if ~activeSyst
                evtViol_ = JumpSet(yGRID_{i_}(:,end), z_(:,i_), p_);
            else
                evtViol_ = JumpSet(yGRID_{i_}(:,end), z_(:,i_), p_, exctFcnHndl, s_);
            end
            ceq_(sum(nPerInterval)*nDIM + nnz(yPERIOD) + nnz(zPERIOD) + i_ ) =...
                evtViol_(ordEvents(i_));
        end
        
        % Additional computations for the Jacobian are done here:
        if nargout>1
            % a) Impact of changes in ySTART_ on the overall periodicity:
            for i_ = find(yOPTIM)'
                % Every entry of ySTART_ changes the corresponding
                % constraint with an derivative of -1:
                if yPERIOD(i_)
                    Jceq_(sum(nPerInterval)*nDIM + nnz(yPERIOD(1:i_)), nnz(yOPTIM(1:i_))) = -1;
                end
            end
            
            % b) Impact of changes to the last element on the overall
            % periodicity:
            for k_ = 1:nDIM
                y_var_ = zeros(nDIM,1);
                y_var_(k_) = var_;
                if ~activeSyst
                    [yFINAL_, zFINAL_] = JumpMap(yGRID_{end}(:,end) + y_var_, z_(:,end), p_, ordEvents(end));
                else
                    [yFINAL_, zFINAL_] = JumpMap(yGRID_{end}(:,end) + y_var_, z_(:,end), p_, exctFcnHndl, s_, ordEvents(end));
                end
                perViol_PLUS_ = [yFINAL_(yPERIOD == 1) - ySTART_(yPERIOD == 1);... 
                                 zFINAL_(zPERIOD == 1) - zSTART_(zPERIOD == 1)];
                Jceq_(sum(nPerInterval)*nDIM + 1  :  sum(nPerInterval)*nDIM + nnz(yPERIOD) + nnz(zPERIOD), OptimizationVariableIndex(nEvents, nPerInterval(end)+1, k_)) =...
                    (perViol_PLUS_ - ceq_(sum(nPerInterval)*nDIM + 1  :  sum(nPerInterval)*nDIM + nnz(yPERIOD) + nnz(zPERIOD)))/var_;
            end
            
            % c) Impact of changes in ySTART_ on the first segment of the
            % first interval: 
            % Compute the values at the end of the first segment:
            y2_ = yGRID_{1}(:,1);
            if ~activeSyst
                f2_ = FlowMap(y2_, zSTART_, p_);
            else
                f2_ = FlowMap(y2_, zSTART_, p_, exctFcnHndl, s_);
            end
            h2_ = tEVENT_(1)/nPerInterval(1);
            for i_ = find(yOPTIM)'
                y_var_ = zeros(nDIM,1);
                y_var_(i_) = var_;
                if ~activeSyst
                    fSTART_ = FlowMap(ySTART_+y_var_, zSTART_, p_);
                else
                    fSTART_ = FlowMap(ySTART_+y_var_, zSTART_, p_, exctFcnHndl, s_);
                end
                Jceq_(1:nDIM,nnz(yOPTIM(1:i_))) = (ComputeDerivativeViolation(ySTART_+y_var_, y2_, fSTART_, f2_, zSTART_, p_, s_, h2_)-ceq_(1:nDIM))/var_;
            end
            
            % d) Impact of changes in the final values of each interval (as
            % these values can influence z, we compute the full numerical
            % derivative):
                
            for i_ = 1:nEvents-1
                % When the 'noChangeInZ' Variable is set to false, it means,
                % that changes to the last y-entry of the previous interval do
                % affect z for the next integration interval, which means that
                % the full influence on the full state vector must be
                % determined (instead of just focusing on the affected
                % segments)
                noChange_ = true;
                for k_ = 1:nDIM
                    y_var_ = zeros(nDIM,1);
                    y_var_(k_) = var_;
                    if ~activeSyst
                        [~, zPLUS_] = JumpMap(yGRID_{i_}(:,end) + y_var_, z_(:,i_), p_, ordEvents(i_));
                    else
                        [~, zPLUS_] = JumpMap(yGRID_{i_}(:,end) + y_var_, z_(:,i_), p_, exctFcnHndl, s_, ordEvents(i_));
                    end
                    if zPLUS_ ~= z_(:,i_+1)
                        noChange_ = false;
                    end
                end
                if noChange_
                    % Changes to yGRID, don't influence the sequence of z.
                    y2_ = yGRID_{i_+1}(:,1);
                    h2_ = (tEVENT_(i_+1)-tEVENT_(i_))/nPerInterval(i_+1);
                    % We can compute f2_ now, as we know that z_ might does
                    % not change due to the event: 
                    if ~activeSyst
                        f2_ = FlowMap(y2_, z_(:,i_+1), p_);
                    else
                        f2_ = FlowMap(y2_, z_(:,i_+1), p_, exctFcnHndl, s_);
                    end
                    for k_ = 1:nDIM
                        % compute the values of the next segment of the next interval:
                        y_var_ = zeros(nDIM,1);
                        y_var_(k_) = var_;
                        if ~activeSyst
                            [yPLUS_, zPLUS_] = JumpMap(yGRID_{i_}(:,end) + y_var_, z_(:,i_), p_, ordEvents(i_));
                        else
                            [yPLUS_, zPLUS_] = JumpMap(yGRID_{i_}(:,end) + y_var_, z_(:,i_), p_, exctFcnHndl, s_, ordEvents(i_));
                        end
                        index_ = DerivativeConstraintIndex(i_+1, 1);
                        if ~activeSyst
                            fPLUS_ = FlowMap(yPLUS_, zPLUS_, p_);
                        else
                            fPLUS_ = FlowMap(yPLUS_, zPLUS_, p_, exctFcnHndl, s_);
                        end
                        Jceq_(index_+1 : index_+nDIM, OptimizationVariableIndex(i_, nPerInterval(i_)+1, k_)) =...
                             (ComputeDerivativeViolation(yPLUS_, y2_, fPLUS_, f2_, zPLUS_, p_, s_, h2_)-ceq_(index_+1 : index_+nDIM))/var_;
                    end
                else
                    % Changes to yGRID, influence the sequence of z, the
                    % states at the end of an integration interval
                    % influence the full simulation:
                    for k_ = 1:nDIM
                        j_ = OptimizationVariableIndex(i_, nPerInterval(i_)+1, k_);
                        x_var_ = zeros(size(x_));
                        x_var_(j_) = var_;
                        Jceq_(:,j_) = (RootTransferFunction(x_ + x_var_)-ceq_)/var_;
                    end
                end
            end
            
            % e) Impact of changes in zINIT, pINIT, and sINIT are computed
            % by finite differences on recursive calls on this function: 
            for i_ = nnz(yOPTIM) +1  :  nnz(yOPTIM) + nnz(zOPTIM) + nnz(pOPTIM) + nnz(sOPTIM) + nEvents 
                x_var_ = zeros(size(x_));
                x_var_(i_) = var_;
                Jceq_(:,i_) = (RootTransferFunction(x_ + x_var_)-ceq_)/var_;
            end
            
            % f) Impact of changes to the last control points of each
            % segment to the event values:
            for i_ = 1:nEvents
                for k_ = 1:nDIM
                    y_var_ = zeros(nDIM,1);
                    y_var_(k_) = var_;
                    if ~activeSyst
                        evtViol_PLUS_ = JumpSet(yGRID_{i_}(:,end)+y_var_, z_(:,i_), p_);
                    else
                        evtViol_PLUS_ = JumpSet(yGRID_{i_}(:,end)+y_var_, z_(:,i_), p_, exctFcnHndl, s_);
                    end
                    Jceq_(sum(nPerInterval)*nDIM + nnz(yPERIOD) + nnz(zPERIOD) + i_, OptimizationVariableIndex(i_, nPerInterval(i_)+1, k_)) = ... 
                    (evtViol_PLUS_(ordEvents(i_))-ceq_(sum(nPerInterval)*nDIM + nnz(yPERIOD) + nnz(zPERIOD) + i_ ))/var_;
                end
            end
        end
    end


	function derViol_ = ComputeDerivativeViolation(yBEGIN_, yEND_, f_yBEGIN_, f_yEND_, z_, p_, s_, h_)
        % Compute the constraint violation of a segment defined by the
        % start and end time, start and end value of states and their
        % derivatives, and the discrete states and system parameters:

        % Compute the vector of cubic parameters 'b__' for the i-th
        % segment: 
        linConst_ = [+1  0 -3  2;
                       0 +1 -2  1;
                       0  0 +3 -2;
                       0  0 -1 +1];
        b_ = [yBEGIN_, h_*f_yBEGIN_, yEND_, h_*f_yEND_]*linConst_;
        % Compute monomial and its derivative (As we're always
        % evaluating the function in the center of the segment this
        % monomials are very simple:  
        yHat_  = [1; 0.5; 0.25; 0.125];
        dyHat_ = [0; 1; 1; 0.75]./h_;
        % this allows us to evaluate the cubic spline at the center, and
        % compare the derivative of its value (given through the
        % FlowMap) with the derivative of the spline:
        if ~activeSyst
            derViol_ = FlowMap(b_*yHat_, z_, p_)  - b_*dyHat_;
        else
            derViol_ = FlowMap(b_*yHat_, z_, p_, exctFcnHndl, s_)  - b_*dyHat_;
        end
    end


    function indexC_ = DerivativeConstraintIndex(i_, j_)
        % Computes the index to a derivative constraint vector for interval
        % 'i_' and segment 'j_'.  The constraint vector will reach from
        % ceq_(index_ + 1 : index_ + nDIM)
        indexC_ = sum(nPerInterval(1:i_-1))*nDIM + (j_-1)*nDIM;
    end


    function indexX_ = OptimizationVariableIndex(i_, j_, k_)
        % Computes the index to an optimization variable in the yGRID for
        % interval 'i_', control point 'j_', and state 'k_'.
        indexX_ = nnz(yOPTIM) + nnz(zOPTIM) + nnz(pOPTIM) + nnz(sOPTIM) + nEvents + sum(nPerInterval(1:i_-1))*nDIM + (j_-2)*nDIM + k_;
    end
    % *********************************************************************
end
% *************************************************************************
% *************************************************************************