classdef nrPDCCHConfig < nr5g.internal.nrPDCCHConfigBase
    %nrPDCCHConfig PDCCH configuration object
    %   CFGPDCCH = nrPDCCHConfig creates a physical downlink control
    %   channel (PDCCH) configuration object. This object specifies the
    %   parameters for the PDCCH as per TS 38.211 Section 7.3.2 and TS
    %   38.213 Section 10.
    %
    %   CFGPDCCH = nrPDCCHConfig(Name,Value) creates a PDCCH configuration
    %   object, CFGPDCCH, with the specified property Name set to the
    %   specified Value. You can specify additional name-value pair
    %   arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPDCCHConfig properties:
    %
    %   NStartBWP            - Starting RB index of the bandwidth part (BWP)
    %                          resource grid relative to CRB 0
    %   NSizeBWP             - Number of resource blocks in BWP
    %   CORESET              - Control resource set configuration object
    %   SearchSpace          - Search space set configuration object
    %   RNTI                 - Radio network temporary identifier
    %   DMRSScramblingID     - PDCCH DM-RS scrambling identity
    %   AggregationLevel     - PDCCH aggregation level {1,2,4,8,16}
    %   AllocatedCandidate   - Candidate used for the PDCCH instance (1-based)
    %   CCEOffset            - Explicit CCE offset (overrides AllocatedCandidate)
    %
    %   Example:
    %   %  Create a default nrPDCCHConfig object.
    %
    %   cfgPDCCH = nrPDCCHConfig;
    %   disp(cfgPDCCH)
    %
    %   See also nrCarrierConfig, nrCORESETConfig, nrSearchSpaceConfig,
    %   nrPDCCH, nrPDCCHResources.

    %   Copyright 2019-2022 The MathWorks, Inc.

    %#codegen

    %   References:
    %   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
    %   Specification Group Radio Access Network; NR; Physical channels and
    %   modulation. Sections 7.3.2, 7.4.1.3.
    %   [2] 3GPP TS 38.331, "3rd Generation Partnership Project; Technical
    %   Specification Group Radio Access Network; NR; Radio Resource
    %   Control (RRC) protocol specification. Section 6.3.2,
    %   PDCCH-Config, PDCCH-ConfigCommon IEs.

    % Public properties
    properties (SetAccess = 'public')

        %NStartBWP Start of BWP resource grid relative to CRB 0
        %   Specify the starting resource block of bandwidth part (BWP)
        %   relative to common resource block 0 (CRB 0) as a nonnegative
        %   integer scalar greater than or equal to
        %   nrCarrierConfig.NStartGrid. The default is 0.
        NStartBWP (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThan(NStartBWP,2474)} = 0;

        %NSizeBWP Number of resource blocks in bandwidth part
        %   Specify the number of resource blocks in the bandwidth part
        %   (BWP) as a scalar positive integer. It must be in the range
        %   1...275 and less than or equal to nrCarrierConfig.NSizeGrid.
        %   The default is 48.
        NSizeBWP (1,1) {mustBeNumeric, mustBeInteger, mustBePositive, mustBeLessThanOrEqual(NSizeBWP,275)} = 48;

        %CORESET Control resource set configuration object
        %   Specify the control resource set (CORESET) configuration as a
        %   scalar <a href="matlab:help('nrCORESETConfig')">nrCORESETConfig</a> object with these properties:
        %
        %   <a href="matlab:help('nrCORESETConfig/CORESETID')">CORESETID</a>           - CORESET ID
        %   <a href="matlab:help('nrCORESETConfig/FrequencyResources')">FrequencyResources</a>  - Allocated frequency resources (6 RBs)
        %   <a href="matlab:help('nrCORESETConfig/Duration')">Duration</a>            - CORESET duration in number of OFDM symbols
        %   <a href="matlab:help('nrCORESETConfig/CCEREGMapping')">CCEREGMapping</a>       - CCE-to-REG mapping, 'interleaved' or 'noninterleaved'
        %   <a href="matlab:help('nrCORESETConfig/REGBundleSize')">REGBundleSize</a>       - Resource-element group (REG) bundle size
        %   <a href="matlab:help('nrCORESETConfig/InterleaverSize')">InterleaverSize</a>     - Interleaver size
        %   <a href="matlab:help('nrCORESETConfig/ShiftIndex')">ShiftIndex</a>          - Shift index
        %
        %   The default is a nrCORESETConfig object with default properties.
        CORESET = nrCORESETConfig;

        %SearchSpace Search space set configuration object
        %   Specify the search space set configuration as a scalar
        %   <a href="matlab:help('nrSearchSpaceConfig')">nrSearchSpaceConfig</a> object with these properties:
        %
        %   <a href="matlab:help('nrSearchSpaceConfig/CORESETID')">CORESETID</a>             - Associated CORESET ID for search space
        %   <a href="matlab:help('nrSearchSpaceConfig/SearchSpaceType')">SearchSpaceType</a>       - Search space type, either 'ue' or 'common'
        %   <a href="matlab:help('nrSearchSpaceConfig/StartSymbolWithinSlot')">StartSymbolWithinSlot</a> - First symbol in slot for monitoring
        %   <a href="matlab:help('nrSearchSpaceConfig/SlotPeriodAndOffset')">SlotPeriodAndOffset</a>   - Monitoring periodicity and offset in slots
        %   <a href="matlab:help('nrSearchSpaceConfig/Duration')">Duration</a>              - Search space duration in slots
        %   <a href="matlab:help('nrSearchSpaceConfig/NumCandidates')">NumCandidates</a>         - Number of candidates per aggregation level
        %
        %  The default is the nrSearchSpaceConfig object with default properties.
        SearchSpace = nrSearchSpaceConfig;

        %RNTI Radio network temporary identifier
        %   Specify the RNTI as a scalar nonnegative integer. RNTI is the
        %   C-RNTI (1...65519) for a PDCCH in a UE-specific search space if
        %   pdcch-DMRS-ScramblingID is configured, or 0 otherwise. The
        %   default is 1.
        RNTI (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(RNTI,65519)} = 1;
    end

    methods
        function obj = nrPDCCHConfig(varargin)
            % Get values from the name-value pairs
            obj@nr5g.internal.nrPDCCHConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow objects as properties to be set to nondefault in codegen
                obj.CORESET = nr5g.internal.parseProp('CORESET', ...
                    nrCORESETConfig,varargin{:});
                obj.SearchSpace = nr5g.internal.parseProp('SearchSpace', ...
                    nrSearchSpaceConfig,varargin{:});
            end
        end

        % Self-validate and set properties
        function obj = set.CORESET(obj,val)
            % CORESET: scalar object only
            prop = 'CORESET';
            validateattributes(val, {'nrCORESETConfig'}, ...
                {'scalar'}, [class(obj) '.' prop], prop);

            obj.(prop) = val;
        end

        function obj = set.SearchSpace(obj,val)
            % SearchSpace: scalar object only
            prop = 'SearchSpace';
            validateattributes(val, {'nrSearchSpaceConfig'}, ...
                {'scalar'}, [class(obj) '.' prop], prop);

            obj.(prop) = val;
        end

       function validateConfig(obj,varargin)
            % validateConfig Validate the nrPDCCHConfig object
            %   Cross-checks among properties and individual object
            %   properties
            %
            %   For INTERNAL use only.
            %
            %   validateConfig(PDCCH, MODE) validates only the subset of dependent
            %   properties as specified by the MODE input. MODE must be one of:
            %       'space'
            %       'resources'

            narginchk(1,2);
            if (nargin==2)
                mode = varargin{1};
            else
                mode = 'full';
            end

            % Common checks
            validateConfig(obj.CORESET);
            validateConfig(obj.SearchSpace);

            % Check RNTI to be zero for common SearchSpaceType
            coder.internal.errorIf(obj.RNTI ~= 0 && ...
                strcmp(obj.SearchSpace.SearchSpaceType, 'common'), ...
                'nr5g:nrPDCCHConfig:InvRNTICSS');

            % Check RNTI to be nonzero for ue SearchSpaceType: disabled
            % as 211 and 213 sections conflict.
%             coder.internal.errorIf(obj.RNTI ~= 0 && ...
%                 strcmp(obj.SearchSpace.SearchSpaceType, 'ue'), ...
%                 'nr5g:nrPDCCHConfig:InvRNTIUE');
            
            baseValidate(obj, mode, obj.CORESET, obj.SearchSpace, obj.NStartBWP, obj.NSizeBWP);
         end
    end
end
