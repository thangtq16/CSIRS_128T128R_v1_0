classdef PUCCHConfigBase < nr5g.internal.wavegen.PXYCHConfigBase & ...
                           nr5g.internal.wavegen.DataSourceCommon
%PUCCHConfigBase Class offering properties common between wavegen PUCCH configuration objects
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.
%
%   PUCCHConfigBase properties (configurable):
%
%   NumUCIBits    - Number of UCI part 1 bits (0...1706) (default 1)
%   DataSourceUCI - Source of UCI part 1 contents (PN or custom) (default 'PN9-ITU')

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

    % Constant, hidden properties
    properties (Constant,Hidden)
        DataSourceUCI_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
    end
    
    % Public, tunable properties
    properties
        %NumUCIBits Number of UCI bits
        % Specify the number of UCI bits as a scalar nonnegative integer.
        % For no UCI transmission, set the value to 0. For formats 0 and 1,
        % NumUCIBits must be in the range 0...2. For formats 2, 3, and 4,
        % NumUCIBits must be in the range 0...1706.
        NumUCIBits (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumUCIBits, 1706)} = 1;
        
        %DataSourceUCI Source of UCI contents
        % Specify DataSourceUCI as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the seed
        % is not specified, then all shift registers are initialized with
        % an active state. This property applies when NumUCIBits > 0. The
        % default is 'PN9-ITU'.
        DataSourceUCI = 'PN9-ITU';
    end

    properties (Dependent = true, Hidden = true)
        NumColumns
        Wpa
    end
    
    methods
        % Self-validate and set properties
        function obj = set.DataSourceUCI(obj,val)
            prop = 'DataSourceUCI';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end

        function numCols = get.NumColumns(obj)
                numCols = obj.getNumCols(1);
        end

        function P = get.Wpa(obj)

            P = obj.calculatePrecodeAndMapMatrix(1);

        end

    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function flag = isInactiveProperty(obj, prop)
            flag = false;
            
            % DataSourceUCI only if NumUCIBits > 0
            if strcmp(prop,'DataSourceUCI')
                flag = ~obj.NumUCIBits;
            end
            
            % SecondHopStartPRB only if FrequencyHopping is not 'neither'
            if strcmp(prop,'SecondHopStartPRB')
                flag = strcmpi(obj.FrequencyHopping,'neither');
            end
        end
    end
end
