classdef OutputCLASS
    % used for the graphical output of the simulations. It defines
    % functions for initilization and regular updating. Specific
    % implementations could reach from simple line graphs to more complex
    % 2D and 3D represtations. 
    %Properties: - 'slowDown' : determines the speed of output. '1' is real
    %                                          time, '0' as fast as possible.
    %                   - 'rate': determines the update rate (in units of simulation time)
    %Methods:   - t = getTimeVector(obj, tStart, tEnd):  Tell the calling
    %                                         function for which point in
    %                                         time output is required.
    %                  - obj = update(obj, y, z, t, u) called to update the
    %                                         output
    properties
        slowDown;
        % this values determines the rate of the output. 1 is realtime.
        %bigger values slow it down according to the given factor, lower
        %values speed it up. 0 means as fast as possible.
        rate;
        % this value determines the time rate at which output is created.
    end
    methods
        function obj = OutputCLASS()
            obj.slowDown = 1; %run in realtime
            obj.rate = 0.04, % with 25fps
        end
        % this function is used to pass the desired refresh rate to the
        % simulation:
        %input:    -the beginning of the simulation 'tStart'
        %             - the end of the simulation 'tEnd'
        %output:  - the time grid vector 't' on which ouput is produced
        %             it will be passed on to the integrator, which in turn
        %             calls the 'update' function.
        function t = getTimeVector(obj, tStart, tEnd)
            tEnd = min(tEnd, 1e3); % limit the time, as infinite integration is not possible
            t       = [tStart:obj.rate:tEnd,tEnd];
            t       = [t(diff(t)~=0),t(end)];
        end
    end
    methods (Abstract)
        % This function is called every time the data has changed. It needs
        % to be  rewritten for evry specific implementation
        % Input: - The continuous state vector y
        %           - The discrete state vector z
        %           - The current time t
        %           - The control state vector u (u = [] for passive system)
        obj = update(obj, y, z, t, u)
    end
    
end
%****************************
%****************************
        %