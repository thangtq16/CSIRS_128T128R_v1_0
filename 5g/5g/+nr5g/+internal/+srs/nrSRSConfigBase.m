classdef (Abstract) nrSRSConfigBase < comm.internal.ConfigBase
    %nrSRSConfigBase Class offering properties common between nrSRSConfig and
    %nrWavegenSRSConfig
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %NumSRSPorts Number of SRS antenna ports
        %   Specify the number of SRS antenna ports. The value must be one
        %   of {1, 2, 4, 8}. The default value is 1.
        NumSRSPorts (1,1) {mustBeMember(NumSRSPorts, [1 2 4 8])} = 1;

        %SymbolStart 0-based index of the first SRS symbol in a slot (L0)
        %   Specify the first OFDM symbol in a slot where the SRS is
        %   transmitted. The SRS must be allocated within the slot
        %   boundaries. The default value is 13.
        SymbolStart (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(SymbolStart, 13)} = 13;

        %NumSRSSymbols Number of SRS symbols
        %   Specify the number of consecutive OFDM symbols in a slot. The
        %   value must be one of {1, 2, 4, 8, 10, 12, 14}. When the
        %   SRSPositioning property is set to true, the values 10 and 14
        %   are not supported and the SubcarrierOffsetTable property of <a
        %   href="matlab: help('nrSRSConfig.SubcarrierOffsetTable')">nrSRSConfig</a>
        %   specifies valid configurations of the NumSRSSymbols and KTC
        %   properties. The default value is 1.
        NumSRSSymbols (1,1) {mustBeMember(NumSRSSymbols, [1 2 4 8 10 12 14])} = 1;
        
        %FrequencyStart Frequency-domain starting position
        %   Specify the index of the first PRB allocated to the SRS in the
        %   frequency domain relative to the carrier resource grid when the
        %   property NRRC = 0. The value must be an integer in the range
        %   0...271. The default value is 0.
        FrequencyStart (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(FrequencyStart, 271)} = 0;
        
        %NRRC Frequency domain additional position in 4-PRB blocks
        %   Specify an additional circular frequency offset to
        %   FrequencyStart in blocks of 4 RBs. The value must be an integer
        %   in the range 0...67. The resulting location of the SRS in the
        %   frequency domain depends on FrequencyStart and the
        %   configuration parameters specified in TS 38.211 Table
        %   6.4.1.4.3-1. The default value is 0.
        NRRC (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NRRC, 67)} = 0;

        %FrequencyScalingFactor Frequency scaling factor
        %   Specify the frequency scaling factor as one of {1, 2, 4} to
        %   reduce the SRS transmission bandwidth for partial frequency
        %   sounding. The frequency scaling factor is denoted as PF in TS
        %   38.211 Section 6.4.1.4.3. The default value (1) disables
        %   frequency scaling.
        FrequencyScalingFactor (1,1) {mustBeMember(FrequencyScalingFactor, [1 2 4])} = 1;

        %EnableStartRBHopping Enable frequency hopping of first resource block
        %   Specify the use of frequency hopping of the initial RB of the
        %   SRS transmission for partial frequency sounding when
        %   FrequencyScalingFactor greater than 1. The default value is false.
        EnableStartRBHopping (1,1) logical = false;

        %StartRBIndex Index of the partial frequency sounding frequency block
        %   Specify the index of the partial frequency sounding block to
        %   control the SRS frequency position when FrequencyScalingFactor
        %   is greater than 1. StartRBIndex must be in the range
        %   0...FrequencyScalingFactor-1. The default value is 0.
        StartRBIndex (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThan(StartRBIndex, 4)} = 0;
        
        %CSRS Row index of the bandwidth configuration table
        %   Specify the bandwidth configuration index to control the
        %   bandwidth allocated to the SRS and the frequency hopping
        %   pattern according to TS 38.211 Table 6.4.1.4.3-1. Larger values
        %   of CSRS result in larger SRS bandwidths. The value must be an
        %   integer in the range 0...63. The default value is 0, which
        %   results in 4-PRB bandwidth.
        CSRS (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(CSRS, 63)} = 0;
        
        %BSRS Column index of the bandwidth configuration table
        %   Specify the bandwidth configuration index to control the
        %   bandwidth allocated to the SRS and the frequency hopping
        %   pattern according to TS 38.211 Table 6.4.1.4.3-1. Larger values
        %   of BSRS result in smaller SRS bandwidths per OFDM symbol. The
        %   value must be an integer in the range 0...3. The default
        %   value is 0.
        BSRS (1,1) {mustBeMember(BSRS, [0 1 2 3])} = 0;
        
        %BHop Frequency hopping index
        %   Specify the frequency hopping index to control the frequency
        %   hopping bandwidth and pattern. The value must be an integer in
        %   the range 0...3. Setting BHop >= BSRS disables frequency
        %   hopping. Larger values of BHop (< BSRS) result in smaller
        %   hopping bandwidths. The default value is 0.
        BHop (1,1) {mustBeMember(BHop, [0 1 2 3])} = 0;
        
        %Repetition Repetition factor in OFDM symbols
        %   Specify the number of consecutive OFDM symbols in a slot for
        %   which the SRS is located in the same frequency resources when
        %   frequency hopping is active. The value must be one of {1, 2, 4,
        %   5, 6, 7, 8, 10, 12, 14}. Repetition must be lower than or equal
        %   to NumSRSSymbols when frequency hopping is active and it is
        %   ignored when frequency hopping is not active. The default value
        %   is 1 (no repetition).
        Repetition (1,1) {mustBeMember(Repetition, [1 2 4 5 6 7 8 10 12 14])} = 1;

        %KTC Transmission comb number in subcarriers
        %   Specify the frequency density of the SRS in subcarriers. The
        %   SRS is allocated every KTC subcarriers. The value of KTC must
        %   be one of {2, 4, 8}. When the SRSPositioning property is set to
        %   true, the SubcarrierOffsetTable property of <a
        %   href="matlab:help('nrSRSConfig.SubcarrierOffsetTable')"
        %   >nrSRSConfig</a> specifies valid configurations of the
        %   NumSRSSymbols and KTC properties. The default value is 2.
        KTC (1,1) {mustBeMember(KTC, [2 4 8])} = 2;
        
        %KBarTC Transmission comb offset in subcarriers
        %   Specify an offset of the transmission comb in subcarriers. The
        %   value must be an integer in the range 0...KTC-1. The default
        %   value is 0.
        KBarTC (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThan(KBarTC, 8)} = 0;
        
        %CyclicShift Cyclic shift number offset (NCS)
        %   The cyclic shift number offset determines the origin of the
        %   cyclic shift applied to the SRS for all antenna ports. The
        %   value must be an integer in the range 0...NCSmax-1, where
        %   NCSmax = 6 for KTC = 8, NCSmax = 12 for KTC = 4, NCSmax = 8 for
        %   KTC = 2. Consecutive cyclic shift numbers (modulo NCSmax) are
        %   used for multiport SRS transmissions. The default value is 0.
        CyclicShift (1,1) {mustBeInteger, mustBeNonnegative, mustBeLessThan(CyclicShift, 12)} = 0;

        %GroupSeqHopping Group or sequence numbers hopping
        %   Specify the type of SRS symbol hopping. The value must be one
        %   of ('neither', 'groupHopping', 'sequenceHopping'). When group or
        %   sequence hopping are enabled, a pseudorandom binary sequence
        %   (PRBS) is used to calculate the sequence and group numbers per
        %   OFDM symbol of the SRS transmission. The default value is 'neither'.
        GroupSeqHopping = nr5g.internal.srs.nrSRSConfigBase.getDefault('GroupSeqHopping');

        %NSRSID Scrambling identity
        %   Specify the scrambling identity of the SRS. The value must be
        %   an integer in the range 0...65535. NSRSID determines the
        %   group number when the GroupSeqHopping property is set to
        %   'neither'. Otherwise, it is employed to initialize the
        %   pseudorandom binary sequence (PRBS) for group or sequence
        %   hopping configurations. The default value is 0.
        NSRSID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NSRSID, 65535)} = 0;
        
        %SRSPositioning Enable SRS for user positioning 
        %   Specify the use of the SRS for positioning. A value of true
        %   corresponds to the higher-layer parameter 'SRS-PosResource-r16'
        %   and false corresponds to 'SRS-Resource'. The default value is
        %   false.
        SRSPositioning (1,1) logical = false;

        %EnableEightPortTDM Enable 8-port time division multiplexing
        %   Specify the use 8-port time division multiplexing (TDM). A
        %   value of true corresponds to the higher-layer parameter
        %   nrofSRS-Ports-n8 being equal to ports8tdm. The default value is
        %   false.
        EnableEightPortTDM (1,1) logical = false;

        %CyclicShiftHopping Enable cyclic shift hopping
        %   Specify the use of cyclic shift hopping. A value of true
        %   corresponds to the higher-layer parameter cyclicShiftHopping
        %   being present. The default value is false.
        CyclicShiftHopping (1,1) logical = false;

        %CyclicShiftHoppingID Cyclic shift hopping identity
        %   Cyclic shift hopping identity specified as a nonnegative
        %   integer. This property initializes the pseudorandom binary
        %   sequence (PRBS) that defines the cyclic shift hopping pattern.
        %   The default value is 0.
        CyclicShiftHoppingID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 0;

        %CyclicShiftHoppingSubset Cyclic shift hopping subset
        %   Set of cyclic shifts used for hopping. This property
        %   corresponds to the higher-layer parameter
        %   cyclicShiftHoppingSubset. A pseudorandom binary sequence (PRBS)
        %   initialized with CyclicShiftHoppingID defines the order of the
        %   cyclic shifts in the hopping pattern. The default value ([])
        %   defines a set {0,1,...,K*NCSmax-1}. NCSmax is the maximum
        %   number of cyclic shifts defined in TS 38.211 Table 6.4.1.4.2-1
        %   and K = 2 when the property HoppingFinerGranularity is true or
        %   K = 1 otherwise.
        CyclicShiftHoppingSubset {mustBeNumeric, mustBeInteger, mustBeNonnegative} = [];

        %HoppingFinerGranularity Enable finer granularity of cyclic shift hopping
        %   Specify the use of finer granularity for cyclic shift hopping.
        %   Set HoppingFinerGranularity to true to specify K = 2 in the
        %   definition of the default cyclic shift hopping subset.
        %   Otherwise, K = 1. A value of true corresponds to the
        %   higher-layer parameter hoppingFinerGranularity being present.
        %   The default value is false.
        HoppingFinerGranularity (1,1) logical = false;

        %CombOffsetHopping Enable comb offset hopping
        %   Specify the use of comb offset hopping. A value of true
        %   corresponds to the higher-layer parameter combOffsetHopping
        %   being present. The default value is false.
        CombOffsetHopping (1,1) logical = false;

        %CombOffsetHoppingID Comb offset hopping identity
        %   Comb offset hopping identity specified as a nonnegative
        %   integer. This property initializes the pseudorandom binary
        %   sequence (PRBS) that controls the comb offset hopping pattern.
        %   The default value is 0.
        CombOffsetHoppingID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 0;

        %CombOffsetHoppingSubset Comb offset hopping subset
        %   Set of comb offsets used for hopping. This property corresponds
        %   to the higher-layer parameter combOffsetHoppingSubset. A
        %   pseudorandom binary sequence (PRBS) initialized with
        %   CombOffsetHoppingID defines the order of the comb offsets in
        %   the hopping pattern. The default value ([]) defines a set
        %   {0,1,...,KTC-1}, where KTC is the transmission comb number.
        CombOffsetHoppingSubset {mustBeNumeric, mustBeInteger, mustBeNonnegative} = [];

        %HoppingWithRepetition Enable comb offset hopping with repetition
        %   Specify the use of time repetition for comb offset hopping.
        %   When set to true, the Repetition property defines the minimum
        %   number of symbols for which the comb offset remains constant. A
        %   value of true corresponds to the higher-layer parameter
        %   hoppingWithRepetition being present. The default value is
        %   false.
        HoppingWithRepetition (1,1) logical = false;

    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        NumSRSPorts_Options     = [1 2 4];
        NumSRSSymbols_Options   = [1 2 4 8 10 12 14];
        SymbolStart_Options     = 0:13;
        KTC_Options             = [2,4,8];
        KBarTC_Options           = 0:7;
        CyclicShift_Options      = 0:11;
        BSRS_Options             = 0:3;
        BHop_Options             = 0:3;
        Repetition_Options       = [1 2 4 5 6 7 8 10 12 14];
        GroupSeqHopping_Values  = {'neither','groupHopping','sequenceHopping'};
        FreqScaling_Options = [1 2 4];
        StartRBIndex_Options = 0:3;
    end

    methods

        % Constructor
        function obj = nrSRSConfigBase(varargin)
            % Set variable-size properties for codegen compatibility
            obj@comm.internal.ConfigBase(...
                'GroupSeqHopping',nr5g.internal.srs.nrSRSConfigBase.getDefault('GroupSeqHopping'),...
                varargin{:}); % Set variable-size properties for codegen compatibility
        end
        
        function obj = set.GroupSeqHopping(obj,val)
            propName = 'GroupSeqHopping';
            validateattributes(val,{'char','string'},...
                {'nonempty'},[class(obj) '.' propName],propName);
            val = validatestring(val,obj.GroupSeqHopping_Values,[class(obj) '.' propName],propName);
            obj.(propName) = ''; % For codegen compatibility
            obj.(propName) = val;
        end
        
    end
    
    methods (Access = public)
        
        % Method to check cross dependencies
        function validateConfig(srs)
            
            % Validate SRS for positioning configuration
            if srs.SRSPositioning
                if any(srs.NumSRSSymbols == [10 14])
                    invalidPositioningConfig = true;
                else
                    offt = nr5g.internal.srs.SRSOffsetK;
                    nSym = [1 2 4 8 12]; % Set of symbols compatible with SRS for positioning 
                    symIdx = (srs.NumSRSSymbols == [0 nSym]);
                    ktcIdx = (srs.KTC == nr5g.internal.srs.nrSRSConfigBase.KTC_Options);
                    invalidPositioningConfig = any(isnan(offt{ktcIdx,symIdx}(1)));                    
                end
                coder.internal.errorIf(invalidPositioningConfig,'nr5g:nrSRS:InconsistentPositioningConfiguration',srs.NumSRSSymbols,srs.KTC);
            end

            % Validate comb offset (KBarTC)
            invalidKBarTC = srs.KBarTC>=srs.KTC;
            coder.internal.errorIf(invalidKBarTC,'nr5g:nrSRS:InconsistentComb',srs.KBarTC,srs.KTC);

            % Validate Repetition
            freqHopping = ( (srs.BHop<srs.BSRS) || ((srs.FrequencyScalingFactor>1) && (srs.EnableStartRBHopping)) );
            invalidRepetition = freqHopping && (srs.Repetition>srs.NumSRSSymbols);
            coder.internal.errorIf(invalidRepetition,'nr5g:nrSRS:InconsistentRepetition',srs.Repetition,srs.NumSRSSymbols);
            
            % Validate cyclic shift number offset (CyclicShift)
            maxcs = [8 12 6];
            kTC = double(srs.KTC);
            nCSmax = maxcs(log2(kTC));
            invalidCS = srs.CyclicShift >= nCSmax;
            coder.internal.errorIf(invalidCS,'nr5g:nrSRS:InconsistentCyclicShift',srs.CyclicShift,nCSmax-1,srs.KTC);

            % Validate StartRBIndex
            PF = double(srs.FrequencyScalingFactor);
            invalidStartRBIndex = (srs.StartRBIndex>=PF) && (PF>1);
            coder.internal.errorIf(invalidStartRBIndex,'nr5g:nrSRS:InconsistentStartRBIndex',srs.StartRBIndex,PF);

            % Validate sequence length is a multiple of 6 if
            % FrequencyScalingFactor > 1
            if PF > 1
                mSRS_BSRS = nr5g.internal.srs.SRSBandwidthConfiguration(srs.CSRS,srs.BSRS);
                NRBsc = 12;
                Msc = mSRS_BSRS*NRBsc/(kTC*PF);

                % Error if the SRS sequence length is not a multiple of 6.
                if mod(Msc,6) ~= 0
                    maxKTC = 8/PF; % Maximum KTC for the current PF
                    maxPF = 8/kTC; % Maximum PF for the current KTC
                    coder.internal.error('nr5g:nrSRS:InconsistentSequenceLength',srs.CSRS,srs.BSRS,srs.KTC,PF,maxKTC,maxPF);
                end
            end

            % Validate 8-port TDM
            if srs.EnableEightPortTDM
                coder.internal.errorIf(srs.NumSRSPorts ~= 8,'nr5g:nrSRS:InconsistentNumSRSPortsTDM',srs.NumSRSPorts);
                coder.internal.errorIf(srs.NumSRSSymbols < 2,'nr5g:nrSRS:InconsistentNumSRSSymbolsTDM',srs.NumSRSSymbols);
            end

        end
        
    end
    
    methods (Static, Access = private)
        % Default values of variable-size char properties. This allows to
        % localize the defaults required for codegen compatibility
        function out = getDefault(propName)
            switch propName
                case 'GroupSeqHopping'
                    out = 'neither';
            end
        end
    end

end
