classdef nrCORESETConfig < comm.internal.ConfigBase
    %nrCORESETConfig Control-resource set (CORESET) configuration object
    %   CRST = nrCORESETConfig creates a CORESET configuration object. The
    %   object contains the parameters for the CORESET used for the
    %   physical downlink control channel (PDCCH) as per TS 38.211 Section
    %   7.3.2.
    %
    %   CRST = nrCORESETConfig(Name,Value) creates a CORESET object,
    %   CRST, with the specified property Name set to the specified
    %   Value. You can specify additional name-value pair arguments in any
    %   order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrCORESETConfig properties:
    %
    %   CORESETID           - CORESET ID
    %   Label               - Alphanumeric description for this CORESET
    %   FrequencyResources  - Allocated frequency resources (6 RBs)
    %   Duration            - CORESET duration in number of OFDM symbols
    %   CCEREGMapping       - CCE-to-REG mapping, 'interleaved' or 'noninterleaved'
    %   REGBundleSize       - Resource-element group (REG) bundle size
    %   InterleaverSize     - Interleaver size
    %   ShiftIndex          - Shift index
    %   PrecoderGranularity - Precoder granularity, 'sameAsREG-bundle' or 'allContiguousRBs'
    %   RBOffset            - RB offset of CORESET start in BWP ([],0...5)
    %   NCCE                - Number of CCE available in the CORESET (read-only)
    %
    %   Example 1:
    %   %  Create a default nrCORESETConfig object.
    %
    %   crst = nrCORESETConfig;
    %   disp(crst)
    %
    %   Example 2:
    %   %  Create an nrCORESETConfig object with interleaved CCE-to-REG
    %   %  mapping and a REG Bundle size of 3 for a duration of 3 OFDM
    %   %  symbols.
    %
    %   crst = nrCORESETConfig('REGBundleSize',3,'Duration',3);
    %   disp(crst)
    %
    %   See also nrPDCCHConfig, nrWavegenPDCCHConfig, nrSearchSpaceConfig, nrPDCCH, nrPDCCHResources.

    %   Copyright 2019-2022 The MathWorks, Inc.

    % References:
    %   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
    %   Specification Group Radio Access Network; NR; Physical channels and
    %   modulation. Sections 7.3.2, 7.4.1.3.
    %   [2] 3GPP TS 38.331, "3rd Generation Partnership Project; Technical
    %   Specification Group Radio Access Network; NR; Radio Resource
    %   Control (RRC) protocol specification. Section 6.3.2,
    %   ControlResourceSet IE.

    %#codegen

    % Public properties
    properties (SetAccess = 'public')

        %CORESETID CORESET ID
        %   Specify the CORESET ID as a nonnegative integer scalar less
        %   than 12. The default is 1. CORESETs with a CORESETID of 0 use
        %   the lowest physical resource block of the CORESET as the
        %   reference point for the DM-RS sequence to subcarrier resource
        %   mapping. All other CORESET ID values use common resource
        %   block 0 for the DM-RS reference point.
        CORESETID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThan(CORESETID, 12)} = 1;

        %Label Custom alphanumeric label
        % Specify Label as a character array or string scalar. Use this
        % property to assign a description to this CORESET object. The
        % default is 'CORESET1'.
        Label = 'CORESET1';
        
        %FrequencyResources Allocated frequency resources
        %   Specify the allocated frequency resources for the CORESET as a
        %   binary vector where each bit corresponds to a group of six
        %   resource blocks (RBs). Grouping starts from the first RB group
        %   in the bandwidth part (BWP). The first (leftmost) bit
        %   corresponds to the first RB group in BWP. The default is
        %   ones(1,8).
        FrequencyResources = ones(1,8);

        %Duration Duration in number of OFDM symbols
        %   Specify the contiguous time duration of the CORESET in number
        %   of OFDM symbols as a scalar from the set of {1,2,3}. The
        %   default is 2.
        Duration (1,1) {mustBeMember(Duration, [1 2 3])} = 2;

        %CCEREGMapping CCE-to-REG mapping
        %   Specify the control-channel elements (CCE) to resource-element
        %   groups (REG) mapping as one of 'interleaved' or
        %   'noninterleaved'. The default is 'interleaved'.
        CCEREGMapping = 'interleaved';

        %REGBundleSize Size of the REG bundles
        %   Specify the size of the resource-element group (REG) bundles as
        %   a scalar from the set of {2,3,6}. The default is 6. This
        %   property only applies when CCEREGMapping is set to
        %   'interleaved'. For noninterleaved mapping, the REG bundle size
        %   is 6.
        REGBundleSize (1,1) {mustBeMember(REGBundleSize, [2 3 6])} = 6;

        %InterleaverSize Interleaver size
        %   Specify the interleaver size for interleaved CCE-to-REG mapping
        %   as a scalar from the set of {2,3,6}. The default is 2. This
        %   property only applies when CCEREGMapping is set to
        %   'interleaved'.
        InterleaverSize (1,1) {mustBeMember(InterleaverSize, [2 3 6])} = 2;

        %ShiftIndex Shift index
        %   Specify the shift index as a nonnegative integer scalar in the
        %   range 0...274 or the physical layer cell identity, NCellID, in
        %   the range 0...1007. The default is 0. This property only
        %   applies when CCEREGMapping is set to 'interleaved'.
        ShiftIndex (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(ShiftIndex, 1007)} = 0;

        %PrecoderGranularity Precoder Granularity 
        %   Specify the precoder granularity associated with CORESET, either
        %   'sameAsREG-bundle' (default) or 'allContiguousRBs'.
        PrecoderGranularity = 'sameAsREG-bundle';

        % RBOffset RB Offset of CORESET in BWP
        %   Specify the RB offset of the first RB of the first possible 6 RB
        %   group to the first RB of the BWP. Use [] (default value) for
        %   Release 15 behaviour where the CORESET frequency resources
        %   start at the first complete group of 6 CRB in the BWP.
        %   Use 0...5 to define the RB offset explicitly (equivalent to rb-Offset-r16).
        RBOffset = [];

    end
    
    properties (SetAccess = 'private')
        %NCCE Number of CCE available in the CORESET
        %   The number of control channel elements (CCE) available for use
        %   in the CORESET (TS 38.213 section 10.1). The value depends on
        %   the number of frequency resources selected and the symbol duration.
        %   One CCE consists of 6 REG, where one REG equals one resource block.
        NCCE;
    end
    
    properties(Constant, Hidden)
        CCEREGMapping_Values = {'interleaved', 'noninterleaved'};
        PrecoderGranularity_Values = {'sameAsREG-bundle', 'allContiguousRBs'};
    end

    methods
        function obj = nrCORESETConfig(varargin)
            % Get value of FrequencyResources from the name-value pairs
            freqRes = nr5g.internal.parseProp('FrequencyResources', ...
                    ones(1,8),varargin{:});

            % Add sets for enum properties with different sized values
            obj@comm.internal.ConfigBase( ...
                'FrequencyResources',freqRes, ...
                'CCEREGMapping','interleaved', ...
                varargin{:});
        end

        % Self-validate and set properties
        function obj = set.Label(obj,val)
          prop = 'Label';    
          validateattributes(val, {'char', 'string'}, {'scalartext'}, ...
              [class(obj) '.' prop], prop);        
          obj.(prop) = convertStringsToChars(val);
        end

        function obj = set.FrequencyResources(obj,val)
            prop = 'FrequencyResources';
            temp = val;
            coder.varsize('temp',[1 45],[0 1]);
            validateattributes(temp,{'numeric'},{'binary','row'}, ...
                [class(obj) '.' prop], prop);

            % Check for max length == 45
            coder.internal.errorIf(length(temp)>45, ...
                'nr5g:nrCORESETConfig:InvFreqResources');

            obj.(prop) = temp;
        end

        function obj = set.CCEREGMapping(obj,val)
            prop = 'CCEREGMapping';
            val = validateEnumProperties(obj, prop, val);

            obj.(prop) = '';   % Signal to codegen that the text length is not constant
            obj.(prop) = val;
        end

        function obj = set.PrecoderGranularity(obj,val)
            prop = 'PrecoderGranularity';
            val = validateEnumProperties(obj, prop, val);

            obj.(prop) = '';   % Signal to codegen that the text length is not constant
            obj.(prop) = val;
        end

        function obj = set.RBOffset(obj,val)
            prop = 'RBOffset';
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);  % Codegen declaration required since the property value can be [], as well as 0...5
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp, {'numeric'},...
                    {'scalar','integer','nonnegative','nonempty','<=',5},...  % rb-Offset-r16 must be 0...5
                    [class(obj) '.' prop], prop);
            end
            obj.(prop) = temp;
        end

        function ncce = get.NCCE(obj)
            % Each frequency resource bitmap bit represents a block of 6 PRB/REG,
            % and 6 PRB/REG equals a CCE
            ncce = sum(obj.FrequencyResources==1)*obj.Duration; 
        end
        
        % Validate the object for cross property validation
        function validateConfig(obj)
            % validateConfig Validate the nrCORESETConfig object.

            if strcmpi(obj.CCEREGMapping,'interleaved')
                % Cross-check Duration and REGBundleSize (L)
                if obj.Duration==3
                    coder.internal.errorIf(obj.REGBundleSize==2, ...
                        'nr5g:nrCORESETConfig:InvREGBForDuration3');
                else
                    coder.internal.errorIf(obj.REGBundleSize==3, ...
                        'nr5g:nrCORESETConfig:InvREGBForDuration2');
                end

                % Cross-check allocated RBs with REGBundlesize (L) and InterleaverSize (R)
                numREGs = 6*obj.NCCE;
                C = numREGs/(double(obj.REGBundleSize)*double(obj.InterleaverSize));
                coder.internal.errorIf(floor(C)~=C, ...
                        'nr5g:nrCORESETConfig:InvInterleaving',numREGs, ...
                        obj.REGBundleSize,obj.InterleaverSize);
            end
        end

        function [f,L] = getCCEMapping(obj)
        % Output the REG bundles ordering, interleaved or not.

            validateConfig(obj);

            % Find the CCE-to-REG mapping
            crstCCEs = obj.NCCE;    % Number of CCE (groups of 6 REG/RB) in CORESET 
            numREGs = 6*crstCCEs;   % Number of REG/RB in CORESET
            if strcmpi(obj.CCEREGMapping,'interleaved')
                L = double(obj.REGBundleSize);
                R = double(obj.InterleaverSize);
                C = double(numREGs)/(L*R);
                f = zeros(R*C,1);       % Interleaved REG bundles
                for cIdx = 0:C-1
                    for rIdx = 0:R-1
                        x = cIdx*R + rIdx;
                        f(x+1) = mod(rIdx*C + cIdx + obj.ShiftIndex, R*C);
                    end
                end
            else % non-interleaved
                % Only L=6, 1 REG Bundle == 1 CCE == 6 RB
                L = 6;
                f = (0:(crstCCEs-1)).';
            end
        end

    end

    methods (Access=protected)
        function flag = isInactiveProperty(obj, prop)
            % Controls the conditional display of properties

            flag = false;

            % REGBundleSize only for interleaved CCEREGMMapping
            if strcmp(prop,'REGBundleSize')
                flag = strcmpi(obj.CCEREGMapping,'noninterleaved');
            end

            % InterleaverSize only for interleaved CCEREGMMapping
            if strcmp(prop,'InterleaverSize')
                flag = strcmpi(obj.CCEREGMapping,'noninterleaved');
            end

            % ShiftIndex only for interleaved CCEREGMMapping
            if strcmp(prop,'ShiftIndex')
                flag = strcmpi(obj.CCEREGMapping,'noninterleaved');
            end

        end
    end

end
