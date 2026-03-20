classdef nrSearchSpaceConfig < comm.internal.ConfigBase
    %nrSearchSpaceConfig Search space set configuration object
    %   CFGSS = nrSearchSpaceConfig creates a search space set
    %   configuration object. The object contains the parameters for the
    %   search space set used for the physical downlink control channel
    %   (PDCCH) as per TS 38.213 Section 10.
    %
    %   CFGSS = nrSearchSpaceConfig(Name,Value) creates a search space set
    %   object, CFGSS, with the specified property Name set to the
    %   specified Value. You can specify additional name-value pair
    %   arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrSearchSpaceConfig properties:
    %
    %   SearchSpaceID         - Search space ID
    %   Label                 - Alphanumeric description for this search space
    %   CORESETID             - Associated CORESET ID for search space
    %   SearchSpaceType       - Search space type, either 'ue' or 'common'
    %   StartSymbolWithinSlot - First symbol of CORESET location in each monitored slot
    %   SlotPeriodAndOffset   - Monitoring periodicity and offset, in slots
    %   Duration              - Search space duration, in slots
    %   NumCandidates         - Number of candidates per aggregation level
    %
    %   Example:
    %   %  Create an nrSearchSpaceConfig object.
    %
    %   cfgSS = nrSearchSpaceConfig;
    %   disp(cfgSS)
    %
    %   See also nrCORESETConfig, nrPDCCHConfig, nrWavegenPDCCHConfig, 
    %   nrPDCCH, nrPDCCHResources.

    %   Copyright 2019-2022 The MathWorks, Inc.

    % References:
    %   [1] 3GPP TS 38.213, "3rd Generation Partnership Project; Technical
    %   Specification Group Radio Access Network; NR; Physical layer
    %   procedures for control. Sections 10, 13.
    %   [2] 3GPP TS 38.331, "3rd Generation Partnership Project; Technical
    %   Specification Group Radio Access Network; NR; Radio Resource
    %   Control (RRC) protocol specification. Section 6.3.2,
    %   SearchSpace IE.

    %#codegen

    % Public properties
    properties (SetAccess = 'public')
        %SearchSpaceID Search space ID
        % Specify SearchSpaceID as a scalar nonnegative integer. The
        % default is 1.
        SearchSpaceID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 1;
        
        %Label Custom alphanumeric label
        % Specify Label as a character array or string scalar. Use this
        % property to assign a description to this search-space
        % configuration object. The default is 'SearchSpace1'.
        Label = 'SearchSpace1';
      
        %CORESETID  Associated CORESET ID for search space
        %   Specify the associated CORESET ID as a nonnegative integer
        %   scalar less than 12. The default is 1.
        CORESETID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThan(CORESETID, 12)} = 1;

        %SearchSpaceType  Search space type
        %   Specify the search space type as one of 'ue' or 'common'. The
        %   default is 'ue'.
        SearchSpaceType = 'ue';

        %StartSymbolWithinSlot  First symbol in slot for monitoring
        %   Specify the first symbol in slot for PDCCH monitoring as a
        %   scalar in range 0...13. The default is 0.
        StartSymbolWithinSlot (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(StartSymbolWithinSlot, 13)} = 0;

        %SlotPeriodAndOffset  Monitoring periodicity and offset in slots
        %   Specify the slot period and offset for PDCCH monitoring as a
        %   two-element integer row vector, [period offset]. The first 
        %   value specifies the period (greater than zero) and the second
        %   value specifies the offset (nonnegative, less than period) with
        %   respect to the period. The default is [1 0].
        SlotPeriodAndOffset = [1 0];

        %Duration Search space duration in slots
        %   Specify the number of consecutive slots that the search space
        %   lasts within each period. The default is 1.
        Duration (1,1) {mustBeNumeric, mustBeInteger, mustBePositive, mustBeLessThan(Duration, 2560)} = 1;

        %NumCandidates  Number of candidates per aggregation level
        %   Specify the number of candidates for each aggregation level as
        %   a five-element vector, where for each aggregation level the
        %   value must be one of {0,1,2,3,4,5,6,8}. The vector element
        %   values correspond to the number of candidates for AL1, AL2,
        %   AL4, AL8, and AL16 respectively. The default is [8 8 4 2 1].
        NumCandidates = [8 8 4 2 1];

    end

    properties(Constant, Hidden)
        SearchSpaceType_Values = {'ue', 'common'};
    end

    methods
        function obj = nrSearchSpaceConfig(varargin)
            % Add sets for enum properties with different sized values
            obj@comm.internal.ConfigBase( ...
                'SearchSpaceType','ue', ...
                varargin{:});
        end
        
         % Self-validate and set properties
        function obj = set.Label(obj,val)
          prop = 'Label';    
          validateattributes(val, {'char', 'string'}, {'scalartext'}, ...
              [class(obj) '.' prop], prop);        
          obj.(prop) = convertStringsToChars(val);
        end

        function obj = set.SearchSpaceType(obj,val)
            prop = 'SearchSpaceType';
            val = validateEnumProperties(obj, prop, val);

            obj.(prop) = '';
            obj.(prop) = val;
        end

        function obj = set.SlotPeriodAndOffset(obj,val)
            prop = 'SlotPeriodAndOffset';

            validateattributes(val,{'numeric'}, {'integer','row'}, ...
                [class(obj) '.' prop], prop);
            coder.internal.errorIf(length(val)~=2, ...
                'nr5g:nrSearchSpaceConfig:InvSlotPeriodLen');

            % Check offset to be < period and nonnegative
            coder.internal.errorIf(val(2) >= val(1) || val(2) < 0, ...
                'nr5g:nrSearchSpaceConfig:InvOffset');

            obj.(prop) = val;
        end

        function obj = set.NumCandidates(obj,val)
            prop = 'NumCandidates';

            validateattributes(val,{'numeric'},{'integer','vector'}, ...
                [class(obj) '.' prop], prop);
            coder.internal.errorIf(length(val)~=5, ...
                'nr5g:nrSearchSpaceConfig:InvNumCandidatesLen');
            coder.internal.errorIf(sum(val)==0, ...
                'nr5g:nrSearchSpaceConfig:NumCandidatesAllZero');
            checkAL(obj,val);

            obj.(prop) = val;
        end

        function validateConfig(obj)
            % validateConfig Validate the nrSearchSpaceConfig object.

            % Check Duration to be less than or equal to slot period
             coder.internal.errorIf(obj.Duration > obj.SlotPeriodAndOffset(1), ...
                'nr5g:nrSearchSpaceConfig:InvDuration', obj.SlotPeriodAndOffset(1));

        end
    end

    methods (Access=protected)
        function flag = isInactiveProperty(~,~)
            % Controls the conditional display of properties

            % All properties are visible.
            flag = false;
        end

        function checkAL(~,val)
            % AL range check {0,1,2,3,4,5,6,8} for NumCandidates
            aggLvls = [1 2 4 8 16];
            for idx = 1:length(aggLvls)
                coder.internal.errorIf(~any(val(idx) == [0 1 2 3 4 5 6 8]), ...
                    'nr5g:nrSearchSpaceConfig:InvNumCandidates',idx,aggLvls(idx));
            end
        end
    end

end