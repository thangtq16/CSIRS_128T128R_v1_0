classdef nrPRSConfig < comm.internal.ConfigBase
    %nrPRSConfig PRS configuration object
    %   CFGPRS = nrPRSConfig creates a positioning reference signal (PRS)
    %   configuration object, CFGPRS, for a PRS resource set with one or
    %   more PRS resources. This object contains the properties related to
    %   TS 38.211 Section 7.4.1.7. By default, the object defines a PRS
    %   resource set with a single PRS resource occupying 52 resource
    %   blocks and spanning over first 12 OFDM symbols in a slot. The PRS
    %   resource configured in the default nrPRSConfig object spans the
    %   full carrier bandwidth, if used in combination with a default
    %   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> object.
    %
    %   CFGPRS = nrPRSConfig(Name,Value) creates a PRS configuration object
    %   with the specified property Name set to the specified Value. You
    %   can specify additional name-value arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPRSConfig properties (configurable):
    %
    %   PRSResourceSetPeriod  - PRS resource set slot periodicity
    %                           (TPRSPeriod) and slot offset (TPRSOffset)
    %                           ('on' (default), 'off', [TPRSPeriod TPRSOffset])
    %   PRSResourceOffset     - Slot offset of each PRS resource relative
    %                           to PRS resource set slot offset (TPRSOffset)
    %                           (0...511) (default 0)
    %   PRSResourceRepetition - PRS resource repetition factor
    %                           (1 (default), 2, 4, 6, 8, 16, 32)
    %   PRSResourceTimeGap    - Slot offset between two consecutive
    %                           repeated instances of a PRS resource
    %                           (1 (default), 2, 4, 8, 16, 32)
    %   MutingPattern1        - Muting bit pattern option-1 (default [])
    %   MutingBitRepetition   - Number of consecutive instances of a PRS
    %                           resource set corresponding to a single
    %                           element of MutingPattern1 binary vector
    %                           (1 (default), 2, 4, 8)
    %   MutingPattern2        - Muting bit pattern option-2 (default [])
    %   NumPRSSymbols         - Number of OFDM symbols allocated for each
    %                           PRS resource (0...12) (default 12)
    %   SymbolStart           - Starting OFDM symbol of each PRS resource
    %                           in a slot (0...13) (default 0)
    %   NumRB                 - Number of physical resource blocks (PRBs)
    %                           allocated for all PRS resources (0...275)
    %                           (default 52)
    %   RBOffset              - Starting PRB index of all PRS resources
    %                           relative to the carrier resource grid
    %                           (0...274) (default 0)
    %   CombSize              - Comb size of all PRS resources
    %                           (2 (default), 4, 6, 12)
    %   REOffset              - Starting resource element (RE) offset in
    %                           the first OFDM symbol of each PRS resource
    %                           (0...CombSize-1) (default 0)
    %   NPRSID                - Sequence identity of each PRS resource
    %                           (0...4095) (default 0)
    %
    %   Constant properties:
    %
    %   FrequencyOffsetTable   - Table containing the relative RE offsets
    %                            in each PRS OFDM symbol defined relative
    %                            to REOffset property, according to
    %                            TS 38.211 Table 7.4.1.7.3-1
    %
    %   Note that the following five properties can be specified as scalars
    %   or vectors, which are unique to each PRS resource in a PRS resource
    %   set:
    %   1. PRSResourceOffset
    %   2. NumPRSSymbols
    %   3. SymbolStart
    %   4. REOffset
    %   5. NPRSID
    %   The number of configured PRS resources is considered as the maximum
    %   of lengths of above five mentioned properties. For the above five
    %   properties, when the value is specified as a vector, the length
    %   must be equal to the number of configured PRS resources. When the
    %   property is specified as a scalar, the same value is used for all
    %   the PRS resources in a PRS resource set.
    %
    %   Example 1:
    %   % Create nrPRSConfig object with its default properties.
    %
    %   prs = nrPRSConfig
    %
    %   Example 2:
    %   % Create a PRS resource set configuration object having two PRS
    %   % resources.
    %
    %   prs = nrPRSConfig;
    %
    %   % Set the properties which are common to all the PRS resources in a
    %   % resource set
    %   prs.PRSResourceSetPeriod = [20 0];
    %   prs.PRSResourceRepetition = 4;
    %   prs.PRSResourceTimeGap = 2;
    %   prs.MutingPattern1 = [1 0];
    %   prs.MutingBitRepetition = 2;
    %   prs.MutingPattern2 = [1 0 1 0];
    %   prs.NumRB = 32;
    %   prs.RBOffset = 10;
    %   prs.CombSize = 4;
    %
    %   % Set the properties which are unique to each PRS resource in a
    %   % resource set (these can be scalars or vectors)
    %   prs.PRSResourceOffset = [0 10];
    %   prs.NumPRSSymbols = [6 4];
    %   prs.SymbolStart = [0 1];
    %   prs.REOffset = 0;
    %   prs.NPRSID = [10 50]
    %
    %   See also nrPRS, nrPRSIndices.

    %   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    properties
        %PRSResourceSetPeriod PRS resource set slot periodicity and slot offset
        %   Specify the slot periodicity and slot offset of a PRS resource
        %   set as one of {'on', 'off', [TPRSPeriod TPRSOffset]}. When the
        %   property is set to 'on', all the PRS resources in the resource
        %   set are assumed to be present in the operating slot. When the
        %   property is set to 'off', all the PRS resources in the resource
        %   set are assumed to be absent in the operating slot. TPRSPeriod
        %   is the resource set slot periodicity and it must be a scalar
        %   positive integer. The nominal value of TPRSPeriod is one of
        %   (2^mu)*[4, 5, 8, 10, 16, 20, 32, 40, 64, 80, 160, 320, 640, 1280, 2560, 5120, 10240],
        %   where mu (0...3) is the subcarrier spacing configuration.
        %   TPRSOffset is the resource set slot offset and is one of
        %   {0,...,TPRSPeriod-1}. The default is 'on'.
        PRSResourceSetPeriod = 'on';

        %PRSResourceOffset Slot offset of each PRS resource (0-based)
        %   Specify the slot offset for each PRS resource as a scalar or a
        %   vector of nonnegative integers in the range 0...511. The value
        %   represents the starting slot offset of a PRS resource relative
        %   to PRS resource set offset (TPRSOffset). In case of vector, the
        %   length must be equal to the number of PRS resources to be
        %   configured in a PRS resource set. In case of scalar, the same
        %   value is applicable to all the PRS resources in a PRS resource
        %   set. It is provided by the higher-layer parameter
        %   dl-PRS-ResourceSlotOffset-r16. Note that this property is
        %   applicable only when PRSResourceSetPeriod is a two-element
        %   vector. The default is 0.
        PRSResourceOffset = 0;

        %PRSResourceRepetition PRS resource repetition factor
        %   Specify the PRS resource repetition factor as a scalar. The
        %   value must be one of {1, 2, 4, 6, 8, 16, 32}, provided by the
        %   higher-layer parameter dl-PRS-ResourceRepetitionFactor-r16.
        %   This value is same for all the PRS resources in a PRS resource
        %   set. Note that this property is applicable only when
        %   PRSResourceSetPeriod is a two-element vector. The default is 1.
        PRSResourceRepetition (1,1) {mustBeMember(PRSResourceRepetition, [1,2,4,6,8,16,32])} = 1;

        %PRSResourceTimeGap Slot offset between two consecutive repeated instances of a PRS resource
        %   Specify the time gap as an offset in terms of number of slots
        %   between two consecutive repeated instances of a PRS resource.
        %   The value must be one of {1, 2, 4, 8, 16, 32}, provided by the
        %   higher-layer parameter dl-PRS-ResourceTimeGap-r16. This value
        %   is same for all the PRS resources in a PRS resource set. Note
        %   that this property is applicable only when PRSResourceSetPeriod
        %   is a two-element vector and PRSResourceRepetition is greater
        %   than 1. The default is 1.
        PRSResourceTimeGap (1,1) {mustBeMember(PRSResourceTimeGap, [1,2,4,8,16,32])} = 1;

        %MutingPattern1 Muting bit pattern option-1
        %   Specify the muting bit pattern option-1 as a binary vector.
        %   Each element in the vector corresponds to a number of
        %   consecutive instances of a PRS resource set based on the
        %   property MutingBitRepetition and indicates whether all the PRS
        %   resources within the PRS resource set instance(s) are
        %   transmitted (binary 1) or muted (binary 0). Use empty ([]) to
        %   disable this muting bit pattern. In case of binary vector other
        %   than [], the length of the vector must be one of
        %   {2, 4, 6, 8, 16, 32}, provided by the higher-layer parameter
        %   mutingOption1-r16. Note that this property is applicable only
        %   when PRSResourceSetPeriod is a two-element vector. The default is [].
        MutingPattern1 = [];

        %MutingBitRepetition Muting bit repetition factor
        %   Specify the muting bit repetition factor as a scalar. The value
        %   indicates the number of consecutive instances of a PRS resource
        %   set (N) corresponding to a single element of MutingPattern1
        %   binary vector. This means that the first element in the
        %   MutingPattern1 corresponds to first N instances of a PRS
        %   resource set, second element corresponds to the next N
        %   instances of a PRS resource set, and so on. The value must be
        %   one of {1, 2, 4, 8}, provided by the higher-layer parameter
        %   dl-PRS-MutingBitRepetitionFactor-r16. Note that this property
        %   is applicable only when PRSResourceSetPeriod is a two-element
        %   vector and MutingPattern1 is other than empty ([]). The default is 1.
        MutingBitRepetition (1,1) {mustBeMember(MutingBitRepetition, [1,2,4,8])} = 1;

        %MutingPattern2 Muting bit pattern option-2
        %   Specify the muting bit pattern option-2 as a binary vector.
        %   Each element in the vector corresponds to a single repetition
        %   index of each PRS resource within an active instance of a PRS
        %   resource set and indicates whether that repetition index of all
        %   the PRS resources is transmitted (binary 1) or muted (binary 0).
        %   First element in the vector corresponds to the first repetition
        %   index, second element corresponds to the second repetition
        %   index, and so on. Use empty ([]) to disable this muting bit
        %   pattern. In case of binary vector other than [], the length of
        %   the vector must be equal to PRSResourceRepetition value, which
        %   is one of {1, 2, 4, 6, 8, 16, 32}. It is provided by the
        %   higher-layer parameter mutingOption2-r16. Note that this
        %   property is applicable only when PRSResourceSetPeriod is a
        %   two-element vector. The default is [].
        MutingPattern2 = [];

        %NumPRSSymbols Number of PRS OFDM symbols
        %   Specify the number of consecutive OFDM symbols allocated for
        %   each PRS resource as a scalar or a vector of nonnegative
        %   integers in the range 0...12. It is provided by the
        %   higher-layer parameter dl-PRS-NumSymbols-r16 and the nominal
        %   value is one of {2, 4, 6, 12}. In case of vector, the length
        %   must be equal to the number of PRS resources to be configured
        %   in a PRS resource set. In case of scalar, the same value is
        %   applicable to all the PRS resources in a PRS resource set. Use
        %   0 to configure empty PRS allocation. The default is 12.
        NumPRSSymbols = 12;

        %SymbolStart Starting OFDM symbol of each PRS resource in a slot (0-based)
        %   Specify the starting OFDM symbol of each PRS resource in a slot
        %   as a scalar or a vector of nonnegative integers in the range
        %   0...13. It is provided by the higher-layer parameter
        %   dl-PRS-ResourceSymbolOffset-r16 and the nominal value is in the
        %   range 0...12. In case of vector, the length must be equal to
        %   the number of PRS resources to be configured in a PRS resource
        %   set. In case of scalar, the same value is applicable to all the
        %   PRS resources in a PRS resource set. The default is 0.
        SymbolStart = 0;

        %NumRB Number of PRBs allocated for all PRS resources
        %   Specify the number of PRBs allocated for all the PRS resources
        %   in a resource set. It must be a scalar nonnegative integer in
        %   the range 0...275. It is provided by the higher-layer parameter
        %   dl-PRS-ResourceBandwidth-r16. Nominal value is in the range
        %   24...272 with a granularity of 4 PRBs. Use 0 to configure empty
        %   PRS allocation. The default is 52.
        NumRB (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumRB,275)} = 52;

        %RBOffset Starting PRB index of all PRS resources relative to carrier resource grid (0-based)
        %   Specify the starting PRB index of all the PRS resources in a
        %   resource set relative to carrier resource grid. It must be a
        %   scalar nonnegative integer in the range 0...274. The default is 0.
        RBOffset (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(RBOffset,274)} = 0;

        %CombSize Comb size of all PRS resources
        %   Specify the comb size of all the PRS resources in a resource
        %   set as a scalar positive integer. Comb size represents the
        %   resource element spacing in each OFDM symbol. The value must be
        %   one of {2, 4, 6, 12}, provided by the higher-layer parameter
        %   dl-PRS-CombSizeN-r16. The default is 2.
        CombSize (1,1) {mustBeMember(CombSize, [2,4,6,12])} = 2;

        %REOffset Starting RE offset in the first PRS OFDM symbol of each PRS resource (0-based)
        %   Specify the resource element (RE) offset in the frequency
        %   domain for the first PRS OFDM symbol of each PRS resource as a
        %   scalar or a vector of nonnegative integers. The relative RE
        %   offsets of the following PRS OFDM symbols are defined relative
        %   to the REOffset value, as defined in TS 38.211 Table 7.4.1.7.3-1.
        %   The value must be one of {0,...,CombSize-1}, provided by the
        %   higher-layer parameter dl-PRS-ReOffset-r16. In case of vector,
        %   the length must be equal to the number of PRS resources to be
        %   configured in a PRS resource set. In case of scalar, the same
        %   value is applicable to all the PRS resources in a PRS resource
        %   set. The default is 0.
        REOffset = 0;

        %NPRSID Sequence identity of each PRS resource
        %   Specify the PRS sequence identity as a scalar or a vector of
        %   nonnegative integers in the range 0...4095, provided by the
        %   higher-layer parameter dl-PRS-SequenceID-r16. In case of
        %   vector, the length must be equal to the number of PRS resources
        %   to be configured in a PRS resource set. In case of scalar, the
        %   same value is applicable to all the PRS resources in a PRS
        %   resource set. The default is 0.
        NPRSID = 0;

    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        PRSPeriod_CharOptions          = {'on','off'};
        MutingPattern1Length_Options   = [2,4,6,8,16,32];
        MutingPattern2Length_Options   = [1,2,4,6,8,16,32];
    end

    % Constant, public properties
    properties (Constant)
        %FrequencyOffsetTable Frequency offsets table
        %   Table containing the relative resource element (RE) offsets in
        %   each PRS OFDM symbol defined relative to REOffset property,
        %   according to TS 38.211 Table 7.4.1.7.3-1.
        FrequencyOffsetTable = getFrequencyOffsetTable;
    end

    methods

        % Constructor
        function obj = nrPRSConfig(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                obj.PRSResourceOffset = nr5g.internal.parseProp('PRSResourceOffset',0,varargin{:});
                obj.SymbolStart = nr5g.internal.parseProp('SymbolStart',0,varargin{:});
                obj.NumPRSSymbols = nr5g.internal.parseProp('NumPRSSymbols',12,varargin{:});
                obj.REOffset = nr5g.internal.parseProp('REOffset',0,varargin{:});
                obj.NPRSID = nr5g.internal.parseProp('NPRSID',0,varargin{:});
            end
        end

        function obj = set.PRSResourceSetPeriod(obj,val)
            prop = 'PRSResourceSetPeriod';
            validateattributes(val,{'numeric','char','string'},{},...
                [class(obj) '.' prop],prop);
            if isnumeric(val)
                temp = val;
                coder.varsize('temp',[2 2],[1 1]);
                validateattributes(temp,{'numeric'},...
                    {'vector','integer','numel',2},[class(obj) '.' prop],prop);
                % Validate the slot periodicity of PRS resource set
                validateattributes(double(temp(1)),{'double'},{'positive'},...
                    [class(obj) '.' prop],['first element of ' prop]);
                % Validate the slot offset of PRS resource set
                validateattributes(double(temp(2)),{'double'},{'nonnegative'},...
                    [class(obj) '.' prop],['second element of ' prop]);
                % Validate the slot offset with respect to the value of
                % slot periodicity
                coder.internal.errorIf(temp(2) >= temp(1),...
                    'nr5g:nrPRS:InvalidResourceSetSlotOffset',temp(2),temp(1));
            else
                temp = validatestring(val,obj.PRSPeriod_CharOptions,...
                    [class(obj) '.' prop],prop);
                obj.(prop) = '';
            end
            obj.(prop) = temp;
        end

        function obj = set.PRSResourceOffset(obj,val)
            prop = 'PRSResourceOffset';
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            validateattributes(temp,{'numeric'},...
                {'vector','integer','nonnegative','<=',511},...
                [class(obj) '.' prop],prop);
            obj.(prop) = temp;
        end

        function obj = set.MutingPattern1(obj,val)
            prop = 'MutingPattern1';
            temp = val;
            coder.varsize('temp',[32 32],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','binary'},...
                    [class(obj) '.' prop], prop);
                mutingPattern1Len = numel(temp);
                flag = ~any(mutingPattern1Len == obj.MutingPattern1Length_Options);
                coder.internal.errorIf(flag,...
                    'nr5g:nrPRS:InvalidMutingPattern1Length',mutingPattern1Len);
            end
            obj.(prop) = temp;
        end

        function obj = set.MutingPattern2(obj,val)
            prop = 'MutingPattern2';
            temp = val;
            coder.varsize('temp',[32 32],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},{'vector','binary'},...
                    [class(obj) '.' prop], prop);
                mutingPattern2Len = numel(temp);
                flag = ~any(mutingPattern2Len == obj.MutingPattern2Length_Options);
                coder.internal.errorIf(flag,...
                    'nr5g:nrPRS:InvalidMutingPattern2Length',mutingPattern2Len);
            end
            obj.(prop) = temp;
        end

        function obj = set.NumPRSSymbols(obj,val)
            prop = 'NumPRSSymbols';
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            validateattributes(temp,{'numeric'},...
                {'vector','integer','nonnegative','<=',12},...
                [class(obj) '.' prop],prop);
            obj.(prop) = temp;
        end

        function obj = set.SymbolStart(obj,val)
            prop = 'SymbolStart';
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            validateattributes(temp,{'numeric'},...
                {'vector','integer','nonnegative','<=',13},...
                [class(obj) '.' prop],prop);
            obj.(prop) = temp;
        end

        function obj = set.REOffset(obj,val)
            prop = 'REOffset';
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            validateattributes(val,{'numeric'},...
                {'vector','integer','nonnegative','<=',11},...
                [class(obj) '.' prop],prop);
            obj.(prop) = temp;
        end

        function obj = set.NPRSID(obj,val)
            prop = 'NPRSID';
            temp = val;
            coder.varsize('temp',[Inf Inf],[1 1]);
            validateattributes(temp, {'numeric'},...
                {'vector','integer','nonnegative','<=',4095},...
                [class(obj) '.' prop], prop);
            obj.(prop) = temp;
        end
    end

    methods(Access = public)
        function out = validateConfig(obj)
            %validateConfig Validates PRS configuration object
            %   OUT = validateConfig(OBJ) validates the inter dependent
            %   properties of specified PRS configuration object OBJ and
            %   returns a structure OUT containing the validated and scalar
            %   expanded properties of PRS.

            % Get the number of values specified for the properties which
            % are unique to a PRS resource
            numResOffVal   = numel(obj.PRSResourceOffset);
            numSymStartVal = numel(obj.SymbolStart);
            numPRSSymVal   = numel(obj.NumPRSSymbols);
            numREOffVal    = numel(obj.REOffset);
            numNPRSIDVal   = numel(obj.NPRSID);
            % Calculate the number of PRS resources configured in a PRS
            % resource set
            numRes = max([numResOffVal, numSymStartVal, numPRSSymVal,...
                numREOffVal, numNPRSIDVal]);

            % PRSResourceOffset
            resOffsetErrFlag = (numResOffVal ~= numRes) && (numResOffVal ~= 1);
            coder.internal.errorIf(resOffsetErrFlag,...
                'nr5g:nrPRS:InvalidPRSResourceOffsetLength',...
                numResOffVal,numRes);
            tempPRSResOff = obj.applyScalarExpansion(obj.PRSResourceOffset,numRes);

            % NumPRSSymbols
            numSymErrFlag = (numPRSSymVal ~= numRes) && (numPRSSymVal ~= 1);
            coder.internal.errorIf(numSymErrFlag,...
                'nr5g:nrPRS:InvalidNumPRSSymbolsLength',...
                numPRSSymVal,numRes);
            tempNumSymb = obj.applyScalarExpansion(obj.NumPRSSymbols,numRes);

            % SymbolStart
            symStartErrFlag = (numSymStartVal ~= numRes) && (numSymStartVal ~= 1);
            coder.internal.errorIf(symStartErrFlag,...
                'nr5g:nrPRS:InvalidSymbolStartLength',...
                numSymStartVal,numRes);
            tempSymbStart = obj.applyScalarExpansion(obj.SymbolStart,numRes);

            % REOffset
            reOffErrFlag = (numREOffVal ~= numRes) && (numREOffVal ~= 1);
            coder.internal.errorIf(reOffErrFlag,...
                'nr5g:nrPRS:InvalidREOffsetLength',numREOffVal,numRes);
            tempREOff = obj.applyScalarExpansion(obj.REOffset,numRes);

            % NPRSID
            prsIDErrFlag = (numNPRSIDVal ~= numRes) && (numNPRSIDVal ~= 1);
            coder.internal.errorIf(prsIDErrFlag,...
                'nr5g:nrPRS:InvalidNPRSIDLength',numNPRSIDVal,numRes);
            tempNPRSID = obj.applyScalarExpansion(obj.NPRSID,numRes);

            % Validate REOffset
            KPRSComb = double(obj.CombSize);
            reOffsetFlag = (tempREOff >= KPRSComb);
            if any(reOffsetFlag(:))
                idx = find(reOffsetFlag,1);
                coder.internal.error('nr5g:nrPRS:InvalidREOffset',...
                    tempREOff(idx(1)),idx(1),KPRSComb);
            end

            % Validate MutingPattern2 length
            TPRSRep = double(obj.PRSResourceRepetition);
            if ~isempty(obj.MutingPattern2)
                mutingPattern2Len = numel(obj.MutingPattern2);
                coder.internal.errorIf(mutingPattern2Len ~= TPRSRep,...
                    'nr5g:nrPRS:InvalidMutingPat2AndPRSResRepComb',...
                    mutingPattern2Len,TPRSRep);
            end

            % Validate the time duration spanned by each PRS resource
            TPRSGap = double(obj.PRSResourceTimeGap);
            if ischar(obj.PRSResourceSetPeriod)
                resSetPeriod = obj.PRSResourceSetPeriod;
            else
                resSetPeriod = double(obj.PRSResourceSetPeriod(:));
                for resIdx = 1:numRes
                    overallOffset = resSetPeriod(2) + tempPRSResOff(resIdx);
                    numSlotsSpannedByRes = TPRSRep + (TPRSRep - 1)*(TPRSGap - 1);
                    resTimeSpan = overallOffset + numSlotsSpannedByRes;
                    timeSpanErrFlag = resTimeSpan > resSetPeriod(1);
                    coder.internal.errorIf(timeSpanErrFlag,...
                        'nr5g:nrPRS:InvalidPRSResourceTimeSpan',...
                        resIdx,resTimeSpan,resSetPeriod(1));
                end
            end

            % Assign the updated PRS properties to the output structure
            out.PRSResourceSetPeriod  = resSetPeriod;
            out.PRSResourceOffset     = tempPRSResOff;
            out.PRSResourceRepetition = TPRSRep;
            out.PRSResourceTimeGap    = TPRSGap;
            out.MutingPattern1        = double(obj.MutingPattern1);
            out.MutingBitRepetition   = double(obj.MutingBitRepetition);
            out.MutingPattern2        = double(obj.MutingPattern2);
            out.NumPRSSymbols         = tempNumSymb;
            out.SymbolStart           = tempSymbStart;
            out.NumRB                 = double(obj.NumRB);
            out.RBOffset              = double(obj.RBOffset);
            out.CombSize              = KPRSComb;
            out.REOffset              = tempREOff;
            out.NPRSID                = tempNPRSID;
            out.FrequencyOffsetValues = getFrequencyOffsetValues;

        end
        function out = applyScalarExpansion(~,val,n)
            %applyScalarExpansion Applies scalar expansion
            %   OUT = applyScalarExpansion(~,VAL,N) returns the scalar
            %   expanded output OUT by repeating the value VAL for N times.
            if numel(val) == 1
                temp = repmat(val,1,n);
            else
                temp = val;
            end
            out = double(temp);
        end
    end
end

% Local functions
function t = getFrequencyOffsetTable
%   T = getFrequencyOffsetTable returns the table T containing the relative
%   resource element offsets in each PRS OFDM symbol defined relative to
%   REOffset property, according to TS 38.211 Table 7.4.1.7.3-1.

    kPrimeVals = getFrequencyOffsetValues;
    columnNames = ["CombSize", "l' = 0", "l' = 1", "l' = 2", "l' = 3", "l' = 4",...
        "l' = 5", "l' = 6", "l' = 7", "l' = 8", "l' = 9", "l' = 10", "l' = 11"];

    % Form the frequency offset table
    t = array2table(kPrimeVals,"VariableNames",columnNames);
    t.Properties.VariableNames = columnNames;
    t.Properties.Description = 'TS 38.211 Table 7.4.1.7.3-1: The frequency offset values k'', as a function of PRS OFDM symbol numbers (l'' = 0...NumPRSSymbols-1)';
end

function kPrimeVals = getFrequencyOffsetValues
%   KPRIMEVALS = getFrequencyOffsetValues returns a matrix KPRIMEVALS
%   containing the relative resource element offsets in all PRS OFDM
%   symbols which are defined relative to REOffset property, according to
%   TS 38.211 Table 7.4.1.7.3-1.

    kPrimeVals = [2   0   1   0   1   0   1   0   1   0   1   0   1;
                  4   0   2   1   3   0   2   1   3   0   2   1   3;
                  6   0   3   1   4   2   5   0   3   1   4   2   5;
                  12  0   6   3   9   1   7   4   10  2   8   5   11];
end