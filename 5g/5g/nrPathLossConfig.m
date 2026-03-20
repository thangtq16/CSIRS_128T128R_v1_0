classdef nrPathLossConfig < comm.internal.ConfigBase
    %nrPathLossConfig NR path loss configuration
    %   PLC = nrPathLossConfig creates a path loss configuration object PLC
    %   for a specific scenario, as described in TR 38.901 Section 7.4.1.
    %   This object contains parameters defining the scenario and path loss
    %   model. By default, the object defines an urban macrocell scenario
    %   with an environment height of 1 m and non optional path loss model.
    %
    %   PLC = nrPathLossConfig(Name,Value) creates a path loss
    %   configuration object with the specified property Name set to the
    %   specified Value. You can specify additional name-value pair
    %   arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPathLossConfig properties:
    %
    %   Scenario            - Scenario ('UMa'(default),'UMi','RMa','InH',
    %                         'InF-SL','InF-DL','InF-SH','InF-DH','InF-HH')
    %   BuildingHeight      - Average height of buildings in meters (5-50) 
    %                         (default 5)
    %   StreetWidth         - Average width of streets in meters (5-50)
    %                         (default 20)
    %   EnvironmentHeight   - Average height of the environment in meters
    %                         (default 1)
    %   OptionalModel       - Use optional path loss model
    %                         (false(default), true)
    %
    %   Example:
    %
    %   % Create path loss configuration for a rural macrocell scenario 
    %   % with an average height of buildings of 7 m and width of streets 
    %   % of 25 m.
    %   plc = nrPathLossConfig;
    %   plc.Scenario = 'RMa';
    %   plc.BuildingHeight = 7;
    %   plc.StreetWidth = 25;
    %
    %   % Configure the carrier frequency, line of sight, and positions of
    %   % BS and UE.
    %   freq = 3.5e9;
    %   los = true;
    %   posBS = [0;0;25];
    %   posUE = [100;100;1.5];
    %   
    %   % Calculate path loss 
    %   pl = nrPathLoss(plc,freq,los,posBS,posUE);
    %
    %   See also nrPathLoss.

    %   Copyright 2021-2023 The MathWorks, Inc.
    
    %#codegen

    properties
      
        %Scenario Scenario        
        %   Specify the scenario as one of {'UMa', 'UMi', 'RMa', 'InH',
        %   'InF-SL','InF-DL','InF-SH','InF-DH','InF-HH'} (TR 38.901 Section 7.2).
        %   The default value is 'UMa'.
        Scenario = 'UMa';
        
        %BuildingHeight Average height of buildings in meters
        %   Specify the average height of buildings in meters in an RMa
        %   scenario. The value must be a positive number in the range
        %   5...50. The default value is 5.
        BuildingHeight (1,1) {mustBeNumeric, mustBeGreaterThanOrEqual(BuildingHeight, 5), mustBeLessThanOrEqual(BuildingHeight, 50)} = 5;
        
        %StreetWidth Average width of streets in meters
        %   Specify the average width of streets in meters in an RMa
        %   scenario. The value must be a positive number in the range
        %   5...50. The default value is 20.
        StreetWidth (1,1) {mustBeNumeric, mustBeGreaterThanOrEqual(StreetWidth, 5), mustBeLessThanOrEqual(StreetWidth, 50)} = 20;
        
        %EnvironmentHeight Average height of the environment in meters
        %   Specify the average height of the environment in meters in UMa
        %   and UMi scenarios. The value must be a real number or
        %   NBS-by-NUE matrix. NBS and NUE are the number of BS and UE. The
        %   default value is 1.
        EnvironmentHeight = 1;
        
        %OptionalModel Use optional path loss model
        %   Specify the use of the optional path loss model, as defined in
        %   TR 38.901 Section 7.4 Table 7.4.1-1 for UMa, UMi, and InH
        %   scenarios. The default value is false.
        OptionalModel (1,1) logical = false;
        
    end
    
    % Constant, hidden properties
    properties (Constant,Hidden)
        
        Scenario_Values   = {'RMa','UMa','UMi','InH','InF-SL','InF-DL','InF-SH','InF-DH','InF-HH'};
        
    end
    
    methods
        % Constructor
        function obj = nrPathLossConfig(varargin)
            % Support name-value pair arguments when constructing object
            obj@comm.internal.ConfigBase(varargin{:});
        end
        function obj = set.Scenario(obj,val)
            prop = 'Scenario';
            obj.(prop) = validatestring(val,obj.Scenario_Values,[class(obj) '.' prop],prop);
        end
        function obj = set.EnvironmentHeight(obj,val)
            prop = 'EnvironmentHeight';
            validateattributes(val,{'numeric'},{'real','nonnan'},[class(obj) '.' prop],prop);
            obj.(prop) = val;
        end
    end

    methods (Access  = protected)
        function inactive = isInactiveProperty(obj, prop)
            % Controls the conditional display of properties
            
            inactive = false;
            switch prop
                case 'EnvironmentHeight'
                    inactive = any(strcmpi(obj.Scenario(1:3),{'RMa','InH','InF'}));
                case {'BuildingHeight','StreetWidth'}
                    inactive = any(strcmpi(obj.Scenario(1:3),{'UMa','UMi','InH','InF'}));
                case 'OptionalModel'
                    inactive = any(strcmpi(obj.Scenario(1:3),{'RMa','InF'}));
            end            
        end
    end
end