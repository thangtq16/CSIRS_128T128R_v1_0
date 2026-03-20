classdef nrPRACHConfig < comm.internal.ConfigBase
    %nrPRACHConfig PRACH configuration object
    %   PRACH = nrPRACHConfig creates a physical random access channel
    %   (PRACH) configuration object for a PRACH preamble. This object
    %   provides the properties related to TS 38.211 Section 5.3.2 and
    %   Section 6.3.3. The default nrPRACHConfig object configures a PRACH
    %   preamble format 0, which is placed at the start of the allocated
    %   resources and is active in all subframes for frequency range 1 and
    %   FDD (paired spectrum).
    %
    %   PRACH = nrPRACHConfig(Name,Value) creates a PRACH configuration
    %   object with the specified property Name set to the specified Value.
    %   You can specify additional name-value pair arguments in any order
    %   as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPRACHConfig properties (configurable):
    %
    %   FrequencyRange       - Frequency range (default 'FR1')
    %   DuplexMode           - Duplex mode (default 'FDD')
    %   ConfigurationIndex   - Configuration index, as defined in TS 38.211
    %                          Tables 6.3.3.2-2 to 6.3.3.2-4 (default 27)
    %   SubcarrierSpacing    - PRACH subcarrier spacing in kHz (default 1.25)
    %   LRA                  - Length of the Zadoff-Chu preamble sequence
    %                          (default 839)
    %   SequenceIndex        - Logical root sequence index (default 0)
    %   PreambleIndex        - Scalar preamble index within cell (default 0)
    %   RestrictedSet        - Type of restricted set (default 'UnrestrictedSet')
    %   ZeroCorrelationZone  - Cyclic shift configuration index (default 0)
    %   RBOffset             - Starting resource block (RB) index of the
    %                          initial uplink bandwidth part (BWP) relative
    %                          to carrier resource grid (default 0)
    %   FrequencyStart       - Frequency offset of lowest PRACH transmission
    %                          occasion in frequency domain with respect to
    %                          PRB 0 of the initial uplink BWP (default 0)
    %   RBSetOffset          - Starting RB index of the uplink RB set for
    %                          this PRACH transmission occasion (default 0)
    %   FrequencyIndex       - Index of the PRACH transmission occasions in
    %                          frequency domain (default 0)
    %   TimeIndex            - Index of the PRACH transmission occasions in
    %                          time domain (default 0)
    %   ActivePRACHSlot      - Active PRACH slot number within a subframe or
    %                          a 60 kHz slot (default 0)
    %   NPRACHSlot           - PRACH slot number (default 0)
    %
    %
    %   nrPRACHConfig properties (read-only):
    %
    %   Format                - Preamble format, as defined in TS 38.211
    %                           Tables 6.3.3.1-1 and 6.3.3.1-2
    %   NumTimeOccasions      - Total allowed number of PRACH occasions in
    %                           time
    %   PRACHDuration         - PRACH duration in OFDM symbols
    %   SymbolLocation        - Location of the first OFDM symbol of the
    %                           current PRACH occasion
    %   SubframesPerPRACHSlot - Subframes per nominal PRACH slot
    %   PRACHSlotsPerPeriod   - PRACH slots per overall period
    %
    %
    %   nrPRACHConfig constant properties:
    %
    %   Tables              - Structure containing PRACH-related tables
    %                         from TS 38.211
    %
    %
    %   Note that you can set NPRACHSlot to values larger than the number
    %   of slots per frame. For example, you can set this value using
    %   transmission loop counters in a MATLAB(R) simulation. In this case,
    %   you may have to ensure that the property value is modulo the number
    %   of slots per frame in a calling code.
    %
    %   In the case of format C0, each preamble has one active sequence
    %   period. The preamble spans two OFDM symbols, including the guard
    %   and the cyclic prefix. For this reason, the grid related to format
    %   C0 has 7 OFDM symbols, rather than 14, and each value related to
    %   OFDM symbols that is derived directly from TS 38.211 is halved. For
    %   this reason, the values of the read-only properties PRACHDuration
    %   and SymbolLocation, in the case of format C0, are half of what it
    %   is expected from TS 38.211 Sections 5.3.2 and 6.3.3.
    %
    %   Example 1:
    %   % Create an nrPRACHConfig object with default properties.
    %
    %   prach = nrPRACHConfig
    %
    %   Example 2:
    %   % Create an nrPRACHConfig object with ConfigurationIndex of 106 and
    %   % subcarrier spacing of 30 kHz. Considering the default values of
    %   % the remaining properties, this configuration corresponds to a
    %   % PRACH preamble format 'A1'.
    %
    %   prach = nrPRACHConfig;
    %   prach.ConfigurationIndex = 106;
    %   prach.SubcarrierSpacing = 30
    %
    %   See also nrPRACH, nrPRACHIndices, nrPRACHGrid.

    %   Copyright 2019-2022 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %FrequencyRange Frequency range, as defined in TS 38.101-1 Table 5.1-1
        %   Specify the frequency range, as defined in TS 38.101-1 Table
        %   5.1-1. FrequencyRange and DuplexMode specify one of the
        %   configuration tables in TS 38.211. Valid combinations are:
        %     - 'FR1' and 'FDD'/'SUL' (paired spectrum/supplementary uplink):
        %       Table 6.3.3.2-2
        %     - 'FR1' and 'TDD' (unpaired spectrum):
        %       Table 6.3.3.2-3
        %     - 'FR2' and 'TDD' (unpaired spectrum):
        %       Table 6.3.3.2-4
        %   Paired spectrum relates to FDD duplex mode. Unpaired spectrum
        %   relates to TDD duplex mode. For more information on how paired
        %   and unpaired spectrums relate to duplex mode, see the field
        %   "FDD-OrSUL" of the higher layer parameter "FrequencyInfoUL" in
        %   TS 38.331 Section 6.3.2.
        %   The default is 'FR1'.
        FrequencyRange = 'FR1';
        
        %DuplexMode Duplex mode for uplink
        %   Specify the duplex mode for uplink. Valid values are:
        %       - 'FDD' : Frequency division duplex
        %       - 'TDD' : Time division duplex
        %       - 'SUL' : Supplementary uplink
        %   DuplexMode and FrequencyRange specify one of the configuration
        %   tables in TS 38.211. Valid combinations are:
        %     - 'FR1' and 'FDD'/'SUL' (paired spectrum/supplementary uplink):
        %       Table 6.3.3.2-2
        %     - 'FR1' and 'TDD' (unpaired spectrum):
        %       Table 6.3.3.2-3
        %     - 'FR2' and 'TDD' (unpaired spectrum):
        %       Table 6.3.3.2-4
        %   Paired spectrum relates to FDD duplex mode. Unpaired spectrum
        %   relates to TDD duplex mode. For more information on how paired
        %   and unpaired spectrums relate to duplex mode, see the field
        %   "FDD-OrSUL" of the higher layer parameter "FrequencyInfoUL" in
        %   TS 38.331 Section 6.3.2.
        %   The default is 'FDD'.
        DuplexMode = 'FDD';
        
        %ConfigurationIndex Configuration index (0...262)
        %   Specify the time resources for transmitting the random access
        %   preamble, according to TS 38.211 Tables 6.3.3.2-2 to 6.3.3.2-4.
        %   The selected configuration table depends on FrequencyRange and
        %   DuplexMode. Specify ConfigurationIndex as a nonnegative scalar
        %   in the range 0...262. For FrequencyRange 'FR1' and DuplexMode
        %   'FDD' or for FrequencyRange 'FR2', ConfigurationIndex must be a
        %   nonnegative scalar in the range 0...255.
        %   The default is 27.
        ConfigurationIndex (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(ConfigurationIndex,0), mustBeLessThanOrEqual(ConfigurationIndex,262)} = 27;
        
        %SubcarrierSpacing Subcarrier spacing in kHz
        %   Specify the subcarrier spacing of the PRACH in kHz as a
        %   scalar positive integer. The value must be one of the set
        %   {1.25, 5, 15, 30, 60, 120, 480, 960}.
        %   The default is 1.25.
        SubcarrierSpacing (1,1) {mustBeNumeric, mustBeMember(SubcarrierSpacing,[1.25,5,15,30,60,120,480,960])} = 1.25;
        
        %SequenceIndex Logical root sequence index (0...1149)
        %   Specify the logical root sequence index, as defined by the
        %   higher layer parameter "prach-RootSequenceIndex" or
        %   "prach-RootSequenceIndex-r16" and referred to as "i" in TS
        %   38.211 Tables 6.3.3.1-3, 6.3.3.1-4, 6.3.3.1-4A, and 6.3.3.1-4B.
        %   The default is 0.
        SequenceIndex (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(SequenceIndex,0), mustBeLessThanOrEqual(SequenceIndex,1149)} = 0;
        
        %PreambleIndex Scalar preamble index within cell (0...63)
        %   Specify the preamble index within a cell, as defined by the
        %   higher layer parameter "ra-PreambleIndex".
        %   The default is 0.
        PreambleIndex (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(PreambleIndex,0), mustBeLessThanOrEqual(PreambleIndex,63)} = 0;
        
        %RestrictedSet Type of restricted set
        %   Specify the type of restricted set, as defined by the higher
        %   layer parameter "restrictedSetConfig". RestrictedSet determines
        %   the value of NCS from TS 38.211 Tables 6.3.3.1-5 to 6.3.3.1-7.
        %   As shown in TS 38.211, it can assume one value amongst
        %   {'UnrestrictedSet', 'RestrictedSetTypeA',
        %   'RestrictedSetTypeB'}.
        %   The default is 'UnrestrictedSet'.
        RestrictedSet = nrPRACHConfig.defaultRestrictedSet;
        
        %ZeroCorrelationZone Cyclic shift configuration index (0...15)
        %   Specify the cyclic shift configuration index, as defined by the
        %   higher layer parameter "zeroCorrelationZoneConfig".
        %   ZeroCorrelationZone together with RestrictedSet and
        %   SubcarrierSpacing determine the number of cyclic shifts for the
        %   sequence generation, as defined in TS 38.211 Tables 6.3.3.1-5
        %   to 6.3.3.1-7.
        %   The default is 0.
        ZeroCorrelationZone (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(ZeroCorrelationZone,0), mustBeLessThanOrEqual(ZeroCorrelationZone,15)} = 0;
        
        %RBOffset Starting RB index of the initial uplink BWP (0...274)
        %   Specify the RB index where the initial uplink BWP starts
        %   relative to the carrier resource grid. It must be an integer
        %   scalar in the range 0...274.
        %   The default is 0.
        RBOffset (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(RBOffset,0), mustBeLessThanOrEqual(RBOffset,274)} = 0;
        
        %FrequencyStart Frequency offset of lowest PRACH transmission occasion in frequency domain with respect to PRB 0 (0...274)
        %   Specify the frequency offset, as defined by the higher layer
        %   parameter "msg1-FrequencyStart" and referred to as "n_RA^start"
        %   in TS 38.211 Section 5.3.2.
        %   The default is 0.
        FrequencyStart (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(FrequencyStart,0), mustBeLessThanOrEqual(FrequencyStart,274)} = 0;
        
        %RBSetOffset Starting RB index of the uplink RB set for this PRACH transmission occasion (0...274)
        %   Specify the RB index of the uplink RB set for this PRACH
        %   transmission occasion. It is defined as the difference between
        %   the start CRB of uplink RB sets related to "n_RA^start + n_RA"
        %   and "n_RA^start", respectively. This property is used in the
        %   computation of the PRACH indices, as discussed in TS 38.211
        %   Section 5.3.2, in the case of FR1 and LRA 571 or 1151. It must
        %   be an integer scalar in the range 0...274.
        %   The default is 0.
        RBSetOffset (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(RBSetOffset,0), mustBeLessThanOrEqual(RBSetOffset,274)} = 0;
        
        %FrequencyIndex Index of the PRACH transmission occasion in frequency domain (0...7)
        %   Specify the frequency index of the PRACH transmission occasion.
        %   It is defined as an integer in the range {0:M-1}, in which M is
        %   the higher layer parameter "msg1-FDM" defined in TS 38.331
        %   Section 6.3.2. M can assume values in {1, 2, 4, 8}.
        %   FrequencyIndex is referred to as "n_RA" in TS 38.211 Sections
        %   5.3.2 and 6.3.3.2. This property is inactive for an FR1 carrier
        %   with LRA = {571, 1151}.
        %   The default is 0.
        FrequencyIndex (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(FrequencyIndex,0), mustBeLessThanOrEqual(FrequencyIndex,7)} = 0;
        
        %TimeIndex Index of the PRACH transmission occasion in time domain
        %   Specify the time index of the PRACH transmission occasion. For
        %   LRA = 839, TimeIndex must be zero. For LRA = {139, 571, 1151},
        %   TimeIndex is defined as an integer in the range {0:N_t^{RA,slot}-1},
        %   in which N_t^{RA,slot} is given in TS 38.211 Tables 6.3.3.2-2
        %   to 6.3.3.2-4. TimeIndex is referred to as "n_t^RA" in TS 38.211
        %   Section 5.3.2.
        %   The default is 0.
        TimeIndex (1,1) {mustBeNumeric, mustBeInteger, ...
            mustBeGreaterThanOrEqual(TimeIndex,0), mustBeLessThanOrEqual(TimeIndex,6)} = 0;
        
        %ActivePRACHSlot Active PRACH slot within a subframe or a 60 kHz slot
        %   Specify the position of the active PRACH slot within a
        %   subframe, for an FR1 carrier, or within a 60 kHz slot, for an
        %   FR2 carrier, as specified by the parameter "n_slot^RA" defined
        %   in TS 38.211 Section 5.3.2. If SubcarrierSpacing is set to
        %   1.25, 5, 15, or 60 kHz, then ActivePRACHSlot must be 0. If
        %   SubcarrierSpacing is set to 30 or 120 kHz, then ActivePRACHSlot
        %   can be 0 or 1. If SubcarrierSpacing is set to 480 kHz, then
        %   ActivePRACHSlot can be 3 or 7. If SubcarrierSpacing is set to
        %   960 kHz, then ActivePRACHSlot can be 7 or 15.
        %   The default value is 0.
        ActivePRACHSlot (1,1) {mustBeNumeric, mustBeMember(ActivePRACHSlot,[0,1,3,7,15])} = 0;
        
        %NPRACHSlot PRACH slot number
        %   Specify the PRACH slot number as a scalar nonnegative integer.
        %   The default value is 0.
        NPRACHSlot (1,1) {mustBeNumeric, mustBeFinite, mustBeInteger, mustBeNonnegative} = 0;
        
    end % Public, tunable properties
    
    % Public, dependent properties
    properties (Dependent)
        %LRA Length of the Zadoff-Chu preamble sequence
        %   Specify the value of the Zadoff-Chu preamble sequence, as
        %   specified by the parameter "L_RA" defined in TS 38.211 Section
        %   6.3.3. For long preambles, LRA must be 839. For short preambles
        %   with SubcarrierSpacing set to 15 kHz, LRA can be 139 or 1151.
        %   For short preambles with SubcarrierSpacing set to 30 or 480
        %   kHz, LRA can be 139 or 571. For short preambles with
        %   SubcarrierSpacing set to 120 kHz, LRA can be 139, 571, or 1151.
        %   For short preambles with SubcarrierSpacing set to 60 or 960
        %   kHz, LRA must be 139.
        %   The default value is 839.
        LRA (1,1) {mustBeNumeric, mustBeMember(LRA,[139,571,839,1151])};
    end % Public, dependent properties
    
    % Read-only properties
    properties (SetAccess = private)
        %Format Preamble format, as defined in TS 38.211 Tables 6.3.3.1-1 and 6.3.3.1-2
        %   Values are {'0','1','2','3','A1','A2','A3','B1','B2','B3','B4','C0','C2'}
        Format;
        
        %NumTimeOccasions Total allowed number of PRACH occasions in time within the PRACH slot
        %   This parameter is referred to as 'N_t^{RA,slot}' in TS 38.211
        %   and its value is given in Tables 6.3.3.2-2 to 6.3.3.2-4.
        %   NumTimeOccasions is set to 1 for long preambles, as specified
        %   in TS 38.211 Section 5.3.2.
        NumTimeOccasions;
        
        %PRACHDuration PRACH duration in OFDM symbols
        %   Number of OFDM symbols in the PRACH slot grid corresponding to
        %   one transmission occasion. In the case of format C0, its value
        %   is half of what is shown in the last column of TS 38.211 Tables
        %   6.3.3.2-2 to 6.3.3.2-4. This is because the grid related to
        %   format C0 has 7 OFDM symbols, instead of 14.
        PRACHDuration;
        
        %SymbolLocation Location of the first OFDM symbol of the current PRACH occasion
        %   This parameter is referred to as 'l' in TS 38.211 Section 5.3.2
        %   and corresponds to the location of the first OFDM symbol of the
        %   current PRACH occasion within a slot. Note that its value can
        %   be outside one PRACH slot, if prach.ActivePRACHSlot is not set
        %   to 0. In the case of format C0, its value is half of what is
        %   expected because the grid related to format C0 has 7 OFDM
        %   symbols, instead of 14.
        SymbolLocation;
        
        %SubframesPerPRACHSlot Subframes per nominal PRACH slot
        %   The total number of subframes spanned by a nominal PRACH slot.
        SubframesPerPRACHSlot;
        
        %PRACHSlotsPerPeriod PRACH slots per overall period
        %   Number of PRACH slots in the overall period that spans an 
        %   integer multiple of 'x' frames, where 'x' is given by TS 38.211 
        %   Tables 6.3.3.2-2 to 6.3.3.2-4.
        PRACHSlotsPerPeriod;
        
    end % Read-only properties
    
    % Constant, hidden properties
    properties (Constant,Hidden)
        FrequencyRange_Values = {'FR1', 'FR2'};
        DuplexMode_Values = {'FDD', 'TDD', 'SUL'};
        RestrictedSet_Values = {'UnrestrictedSet', 'RestrictedSetTypeA', 'RestrictedSetTypeB'};
        CustomPropList = {'FrequencyRange', 'DuplexMode', 'ConfigurationIndex', ...
            'SubcarrierSpacing', 'LRA', 'SequenceIndex', 'PreambleIndex', 'RestrictedSet', ...
            'ZeroCorrelationZone', 'RBOffset', 'FrequencyStart', 'RBSetOffset', 'FrequencyIndex', ...
            'TimeIndex', 'ActivePRACHSlot', 'NPRACHSlot', ...
            'Format', 'NumTimeOccasions', 'PRACHDuration', 'SymbolLocation', ...
            'SubframesPerPRACHSlot', 'PRACHSlotsPerPeriod', ...
            'Tables'};
    end % Constant, hidden properties
    
    % Constant properties
    properties(Constant)
        %Tables Structure containing PRACH-related tables from TS 38.211
        %   This property contains these tables from TS 38.211 Section
        %   6.3.3:
        %       - LongPreambleFormats           - Table 6.3.3.1-1
        %       - ShortPreambleFormats          - Table 6.3.3.1-2
        %       - NCSFormat012                  - Table 6.3.3.1-5
        %       - NCSFormat3                    - Table 6.3.3.1-6
        %       - NCSFormatABC                  - Table 6.3.3.1-7
        %       - SupportedSCSCombinations      - Table 6.3.3.2-1
        %       - ConfigurationsFR1PairedSUL    - Table 6.3.3.2-2
        %       - ConfigurationsFR1Unpaired     - Table 6.3.3.2-3
        %       - ConfigurationsFR2             - Table 6.3.3.2-4
        Tables = struct('LongPreambleFormats', nr5g.internal.prach.getTable6331x(1), ...        % Table 6.3.3.1-1
                        'ShortPreambleFormats', nr5g.internal.prach.getTable6331x(2), ...       % Table 6.3.3.1-2
                        'NCSFormat012', nr5g.internal.prach.getTable6331x(5), ...               % Table 6.3.3.1-5
                        'NCSFormat3', nr5g.internal.prach.getTable6331x(6), ...                 % Table 6.3.3.1-6
                        'NCSFormatABC', nr5g.internal.prach.getTable6331x(7), ...               % Table 6.3.3.1-7
                        'SupportedSCSCombinations', nr5g.internal.prach.getTable6332x(1), ...   % Table 6.3.3.2-1
                        'ConfigurationsFR1PairedSUL', nr5g.internal.prach.getTable6332x(2), ... % Table 6.3.3.2-2
                        'ConfigurationsFR1Unpaired', nr5g.internal.prach.getTable6332x(3), ...  % Table 6.3.3.2-3
                        'ConfigurationsFR2', nr5g.internal.prach.getTable6332x(4));             % Table 6.3.3.2-4
    end % Constant properties
    
    % Private, hidden properties
    properties(Access = private, Hidden)
        theLRA = 839;
    end
    
    % Constructor and Set public methods
    methods

        % Constructor
        function obj = nrPRACHConfig(varargin)
            %nrPRACHConfig Create nrPRACHConfig object
            %   Set the property values from any name-value pairs input to
            %   the object
            obj@comm.internal.ConfigBase(...
                'RestrictedSet',nrPRACHConfig.defaultRestrictedSet,...
                varargin{:});
        end
        
        % Self-validate and set properties
        function obj = set.FrequencyRange(obj,val)
            prop = 'FrequencyRange';
            validateattributes(val,{'char','string'},...
                {'nonempty'},[class(obj) '.' prop], prop);
            temp = validatestring(val,obj.FrequencyRange_Values,[class(obj) '.' prop],prop);
            obj.(prop) = temp(1:3);
            obj = getLRA(obj);
        end
        
        function obj = set.DuplexMode(obj,val)
            prop = 'DuplexMode';
            validateattributes(val,{'char','string'},...
                {'nonempty'},[class(obj) '.' prop], prop);
            temp = validatestring(val,obj.DuplexMode_Values,[class(obj) '.' prop],prop);
            obj.(prop) = temp(1:3);
            obj = getLRA(obj);
        end

        function obj = set.ConfigurationIndex(obj,val)
            % Set the value of ConfigurationIndex and update the internal
            % representation of LRA
            obj.ConfigurationIndex = val; % val is already validated
            obj = getLRA(obj);
        end
        
        function obj = set.SubcarrierSpacing(obj,val)
            % Set the value of SubcarrierSpacing and update the internal
            % representation of LRA
            obj.SubcarrierSpacing = val; % val is already validated
            obj = getLRA(obj);
        end
        
        function obj = set.LRA(obj,val)
            % Update the internal representation of LRA
            obj = setLRA(obj,val);
        end
        
        function obj = set.RestrictedSet(obj,val)
            prop = 'RestrictedSet';
            validateattributes(val,{'char','string'},...
                {'nonempty'},[class(obj) '.' prop], prop);
            temp = validatestring(val,obj.RestrictedSet_Values,[class(obj) '.' prop],prop);
            obj.(prop) = ''; % For codegen compatibility
            obj.(prop) = temp;
        end
        
    end % Constructor and Set public methods
    
    % Get public methods - Read-only properties
    methods
        
        function out = get.Format(obj)
            % Format is updated based on FrequencyRange, DuplexMode,
            % ConfigurationIndex, and TimeIndex, as discussed in TS 38.211
            % Tables 6.3.3.2-2 to 6.3.3.2-4 and Section 5.3.2 (for mixed
            % preamble formats A1/B1, A2/B2, and A3/B3).
            
            params = nr5g.internal.prach.getVariablesFromConfigTable(obj.FrequencyRange,obj.DuplexMode,obj.ConfigurationIndex);
            out = params.Format;
            
            if length(out)==5 % Mixed A/B format
                if obj.TimeIndex == obj.NumTimeOccasions-1
                    out = out(4:5); % B format is considered for the last PRACH occasion in time
                else
                    out = out(1:2); % A format is considered for all but the last PRACH occasion in time
                end
            end
        end
        
        function out = get.LRA(obj)
            % LRA is updated based on Format:
            % TS 38.211 Tables 6.3.3.1-1 and 6.3.3.1-2
            
            out = obj.theLRA;
        end
        
        function out = get.NumTimeOccasions(obj)
            % NumTimeOccasions is updated based on FrequencyRange,
            % DuplexMode, ConfigurationIndex, and Format, as shown in TS 38.211
            % Tables 6.3.3.2-2 to 6.3.3.2-4.
            
            params = nr5g.internal.prach.getVariablesFromConfigTable(obj.FrequencyRange,obj.DuplexMode,obj.ConfigurationIndex);
            out = params.NumTimeOccasions;
            
            if isnan(out)
                % For long sequences (LRA=839) the column "number of
                % time-domain PRACH occasions within a PRACH slot" in
                % the configuration tables in TS 38.211 is marked as "-",
                % whereas it is marked as NaN in the tables stored in this
                % configuration object. Note that the value used here is 1,
                % because there is one long sequence preamble per PRACH
                % slot, as discussed in TS 38.211 Section 5.3.2.
                out = 1;
            end
        end
        
        function out = get.PRACHDuration(obj)
            % PRACHDuration is updated based on Format, SubcarrierSpacing,
            % FrequencyRange, DuplexMode, and ConfigurationIndex:
            % TS 38.211 Tables 6.3.3.1-1, 6.3.3.2-2 to 6.3.3.2-4
            
            format = obj.Format;
            switch format
                case {'0','1','2','3'}
                    % For long sequences (LRA=839) the number of
                    % OFDM symbols per PRACH slot appears as parts of the
                    % expressions for N_u in TS 38.211 Table 6.3.3.1-1:
                    % For format 0: 1
                    % For format 1: 2
                    % For format 2: 4
                    % For format 3: 4
                    table = nr5g.internal.prach.getTable6331x(1);
                    % Codegen does not know that N_u_vector always has
                    % exactly 1 element. Help it by indexing with (1).
                    N_u_vector = table.N_u(strcmpi(table.Format,format));
                    N_u = N_u_vector(1);
                    n_idft_nominal = 2048 * 15 / double(obj.SubcarrierSpacing);
                    out = N_u / n_idft_nominal;
                otherwise
                    % For short sequences (LRA={139,571,1151}), look up the
                    % value from the configuration tables TS 38.211 Tables
                    % 6.3.3.2-2 to 6.3.3.2-4
                    params = nr5g.internal.prach.getVariablesFromConfigTable(obj.FrequencyRange,obj.DuplexMode,obj.ConfigurationIndex);
                    out = params.PRACHDuration;
            end
            
            % In the case of format C0, each preamble has one active
            % sequence period (see TS 38.211 Table 6.3.3.1-2) but including
            % the guard and the cyclic prefix, the preamble spans two OFDM
            % symbols (given by PRACHDuration in the configuration tables
            % above). For this reason, the slot grid related to format C0
            % has 7 OFDM symbols, rather than 14, and each value related to
            % it that is derived directly from TS 38.211 is halved
            if strcmpi(format,'C0')
                out = out / 2;
            end
        end
        
        function out = get.SymbolLocation(obj)
            % SymbolLocation is updated based on FrequencyRange,
            % DuplexMode, ConfigurationIndex, TimeIndex, PRACHDuration, and
            % ActivePRACHSlot, as described in TS 38.211 Section 5.3.2
            
            params = nr5g.internal.prach.getVariablesFromConfigTable(obj.FrequencyRange,obj.DuplexMode,obj.ConfigurationIndex);
            
            % In the case of format C0, each preamble has one active
            % sequence period (see TS 38.211 Table 6.3.3.1-2) but including
            % the guard and the cyclic prefix, the preamble spans two OFDM
            % symbols (given by PRACHDuration in the configuration tables
            % above). For this reason, the slot grid related to format C0
            % has 7 OFDM symbols, rather than 14, and each value related to
            % it that is derived directly from TS 38.211 is halved.
            if strcmpi(obj.Format,'C0')
                startingSymbol = params.StartingSymbol / 2;
            else
                startingSymbol = params.StartingSymbol;
            end

            % Get the number of OFDM symbols per slot
            % Note that numOFDMSymbPerSlot does not influence the value of
            % SymbolLocation for those cases in which ActivePRACHSlot is
            % zero.
            numOFDMSymbPerSlot = nr5g.internal.prach.gridSymbolSize(obj);
            
            % Get the value of SymbolLocation
            out = startingSymbol + (double(obj.TimeIndex) * obj.PRACHDuration) + (numOFDMSymbPerSlot * double(obj.ActivePRACHSlot));
            
            % There are a few exceptions in TS 38.211 Table 6.3.3.2-3 in
            % which a long preamble format is characterized by
            % StartingSymbol with a value of 7. Since long preambles span
            % at least 1 ms, such a value of StartingSymbol generates a
            % PRACH preamble that starts in the middle of one subframe and
            % spans multiple subframes. However, this does not comply with
            % the definition of OFDM symbol used here. For instance, a
            % PRACH preamble format 0 spans one OFDM symbol that lasts 1
            % ms. Thus, the resource grid is characterized by a single OFDM
            % symbol. For this reason, the exceptions above are treated
            % here as if they had a value of StartingSymbol of 0. This is
            % different from what is stated in the standard but the OFDM
            % modulation takes care of it in the time-sequence generation.
            if any(strcmpi(obj.Format,{'0','1','2','3'})) && startingSymbol>0
                out = 0;
            end
        end
        
        function out = get.SubframesPerPRACHSlot(obj)
            
            format = obj.Format;
            if (any(strcmpi(format,{'0','1','2','3'}))) % LRA=839
                table = nr5g.internal.prach.getTable6331x(1);
                N_u = table.N_u(strcmpi(table.Format,format));
                N_CP = table.N_CP(strcmpi(table.Format,format));
                samplesPerSubframe = 30720; % in units of T_s
                out = ceil((N_u + N_CP) / samplesPerSubframe);
            else % LRA={139,571,1151}
                out = 15 / double(obj.SubcarrierSpacing);
            end
            
        end
        
        function out = get.PRACHSlotsPerPeriod(obj)
            
            subframesPerSlot = obj.SubframesPerPRACHSlot;
            params = nr5g.internal.prach.getVariablesFromConfigTable(obj.FrequencyRange,obj.DuplexMode,obj.ConfigurationIndex);
            if ~isnan(params.x)
                out = lcm(params.x*10,round(max([1 subframesPerSlot]))) / subframesPerSlot;
            else
                out = NaN;
            end
            
        end
        
    end % Get public methods - Read-only properties
    
    % Public methods
    methods
        
        function out = validateConfig(obj)
            %validateConfig Validate the nrPRACHConfig object
            %   OUT = validateConfig(OBJ) validates the inter dependent
            %   properties of specified nrPRACHConfig configuration object
            %   and returns one structure OUT with the PRACH parameters.
            
            % Reassign the properties to the out structure
            % Configurable properties
            out.FrequencyRange            = obj.FrequencyRange;
            out.DuplexMode                = obj.DuplexMode;
            out.ConfigurationIndex        = double(obj.ConfigurationIndex);
            out.SubcarrierSpacing         = double(obj.SubcarrierSpacing);
            out.LRA                       = double(obj.LRA);
            out.SequenceIndex             = double(obj.SequenceIndex);
            out.PreambleIndex             = double(obj.PreambleIndex);
            out.RestrictedSet             = obj.RestrictedSet;
            out.ZeroCorrelationZone       = double(obj.ZeroCorrelationZone);
            out.RBOffset                  = double(obj.RBOffset);
            out.FrequencyStart            = double(obj.FrequencyStart);
            out.FrequencyIndex            = double(obj.FrequencyIndex);
            out.RBSetOffset               = double(obj.RBSetOffset);
            out.TimeIndex                 = double(obj.TimeIndex);
            out.ActivePRACHSlot           = double(obj.ActivePRACHSlot);
            out.NPRACHSlot                = double(obj.NPRACHSlot);
            % Read-only properties
            out.Format                    = obj.Format;
            out.NumTimeOccasions          = obj.NumTimeOccasions;
            out.PRACHDuration             = obj.PRACHDuration;
            out.SymbolLocation            = obj.SymbolLocation;
            out.SubframesPerPRACHSlot     = obj.SubframesPerPRACHSlot;
            out.PRACHSlotsPerPeriod       = obj.PRACHSlotsPerPeriod;
            
            % Validation
            checkFRAndDuplexMode(obj,out); % Check for compatibility between FrequencyRange and DuplexMode
            checkCIAndFRAndDuplexMode(obj,out); % Check for compatibility between FrequencyRange, DuplexMode, and ConfigurationIndex
            checkSCSAndFR(obj,out); % Check for compatibility between FrequencyRange and SubcarrierSpacing
            checkSCSAndFormat(obj,out); % Check for compatibility between SubcarrierSpacing and Format
            checkRestrictedSetAndFormat(obj,out); % Check for compatibility between Format and RestrictedSet
            checkAPSAndSCSAndCI(obj,out); % Check for compatibility between ConfigurationIndex, SubcarrierSpacing, and ActivePRACHSlot
            checkTimeIndexAndCI(obj,out); % Check for compatibility between TimeIndex and ConfigurationIndex
        end
        
    end % Public methods
    
    % Cross-parameter checks
    methods(Access = private)
        
        function checkFRAndDuplexMode(~,prach)
            % Checks for the valid combination of FrequencyRange (FR) and
            % DuplexMode. As described in TS 38.104 Tables 5.2-1 and 5.2-2,
            % only 'TDD' duplex mode is allowed for FrequencyRange set
            % to 'FR2'.
            
            errorFlag = strcmpi(prach.FrequencyRange,'FR2') && ~strcmpi(prach.DuplexMode,'TDD');
            coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidFRAndDuplexMode',prach.FrequencyRange,prach.DuplexMode);
        end
        
        function checkCIAndFRAndDuplexMode(~,prach)
            % Checks for the valid combination of ConfigurationIndex (CI),
            % FrequencyRange (FR), and DuplexMode. Values of CI greater
            % than 255 are allowed only for FR1 and TDD (i.e., TS 38.211
            % Table 6.3.3.2-3)
            
            errorFlag = prach.ConfigurationIndex > 255 && ~(strcmpi(prach.FrequencyRange,'FR1') && strcmpi(prach.DuplexMode,'TDD'));
            coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidCIAndFRAndDuplexMode',...
                prach.ConfigurationIndex,prach.FrequencyRange,prach.DuplexMode);
        end
        
        function checkSCSAndFR(~,prach)
            % Checks for the valid combination of FrequencyRange (FR) and
            % SubcarrierSpacing (SCS). From TS 38.211 Table 6.3.3.2-1, we
            % assume that [60, 120, 480, 960] kHz are the only values
            % allowed for PRACH subcarrier spacing in FR2. On the other
            % hand, [1.25, 5, 15, 30] kHz are the only values allowed for
            % PRACH subcarrier spacing in FR1.
            
            errorFlagFR1 = strcmpi(prach.FrequencyRange,'FR1') && prach.SubcarrierSpacing>=60;
            errorFlagFR2 = strcmpi(prach.FrequencyRange,'FR2') && prach.SubcarrierSpacing<60;
            coder.internal.errorIf(errorFlagFR1,'nr5g:nrPRACHConfig:InvalidSCSForFR1',sprintf('%g',prach.SubcarrierSpacing));
            coder.internal.errorIf(errorFlagFR2,'nr5g:nrPRACHConfig:InvalidSCSForFR2',sprintf('%g',prach.SubcarrierSpacing));
        end
        
        function checkSCSAndFormat(~,prach)
            % Checks for the valid combination of SubcarrierSpacing (SCS)
            % and Format, as described in TS 38.211 Tables 6.3.3.1-1 and
            % 6.3.3.1-2.
            
            switch prach.Format
                case {'0','1','2'} % Long preamble
                    SCS = 1.25;
                    errorFlag = prach.SubcarrierSpacing ~= SCS;
                    coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidSCSForFormatLong',sprintf('%g',prach.SubcarrierSpacing),prach.Format,sprintf('%g',SCS));
                case {'3'} % Long preamble
                    SCS = 5;
                    errorFlag = prach.SubcarrierSpacing ~= SCS;
                    coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidSCSForFormatLong',sprintf('%g',prach.SubcarrierSpacing),prach.Format,sprintf('%g',SCS));
                otherwise % Short preamble
                    errorFlag = ~any(prach.SubcarrierSpacing == [15, 30, 60, 120, 480, 960]);
                    coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidSCSForFormatShort',sprintf('%g',prach.SubcarrierSpacing),prach.Format);
            end
        end
        
        function checkRestrictedSetAndFormat(~,prach)
            % Checks for the valid combination of RestrictedSet and Format,
            % as described in TS 38.211 Tables 6.3.3.1-5 to 6.3.3.1-7.
            
            coder.varsize('restrictedSet',[1 18],[0 1]);
            coder.varsize('format',[1 2],[0 1]);
            restrictedSet = prach.RestrictedSet;
            format = prach.Format;
            
            switch format
                case {'0','1','2'} % Long preamble
                    coder.internal.errorIf((strcmpi(restrictedSet,'RestrictedSetTypeA')&&prach.ZeroCorrelationZone==15),...
                        'nr5g:nrPRACHConfig:InvalidZeroCorrelationZone012A',sprintf('%g',prach.ZeroCorrelationZone));
                    coder.internal.errorIf((strcmpi(restrictedSet,'RestrictedSetTypeB')&&prach.ZeroCorrelationZone>12),...
                        'nr5g:nrPRACHConfig:InvalidZeroCorrelationZone012B',sprintf('%g',prach.ZeroCorrelationZone));
                case {'3'} % Long preamble
                    coder.internal.errorIf((strcmpi(restrictedSet,'RestrictedSetTypeB')&&prach.ZeroCorrelationZone>13),...
                        'nr5g:nrPRACHConfig:InvalidZeroCorrelationZone3B',sprintf('%g',prach.ZeroCorrelationZone));
                otherwise % Short preamble
                    coder.internal.errorIf(~strcmpi(restrictedSet,'UnrestrictedSet'),'nr5g:nrPRACHConfig:InvalidRestrictedSetShort',restrictedSet,format);
            end
        end
        
        function checkAPSAndSCSAndCI(~,prach)
            % Checks for the valid combination of ActivePRACHSlot (APS),
            % subcarrier spacing (SCS) and ConfigurationIndex (CI), as
            % described in TS 38.211 Section 5.3.2.
            
            scs = prach.SubcarrierSpacing;
            aps = prach.ActivePRACHSlot;
            params = nr5g.internal.prach.getVariablesFromConfigTable(prach.FrequencyRange,prach.DuplexMode,prach.ConfigurationIndex);
            slotsPerSF = params.SlotsPerSF;
            switch scs
                case {1.25, 5, 15, 60}
                    % ActivePRACHSlot must be 0
                    errorFlag = logical(aps);
                    validAPS = 0;
                otherwise % {30, 120, 480, 960}
                    % ActivePRACHSlot depends on TS 38.211 Tables 6.3.3.2-2
                    % to 6.3.3.2-4 and can be:
                    % * [0,1] for 30 or 120 kHz
                    % * [3,7] for 480 kHz
                    % * [7,15] for 960 kHz
                    if any(scs==[30,120])
                        validAPS = [0,1];
                    elseif scs==480
                        validAPS = [3,7];
                    else % 960
                        validAPS = [7,15];
                    end
                    if slotsPerSF==1
                        validAPS = validAPS(2);
                    end
                    errorFlag = any(~isnan(slotsPerSF) & all(aps ~= validAPS));
            end
            if any(isnan(slotsPerSF)) || isscalar(validAPS)
                coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidAPSForCIAndSCS',sprintf('%d',int8(aps)),sprintf('%d',int8(validAPS(1))));
            else
                coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidAPSForCIAndSCSMultiple',sprintf('%d',int8(aps)),sprintf('%d',int8(validAPS(1))),sprintf('%d',int8(validAPS(2))));
            end

        end
        
        function checkTimeIndexAndCI(~,prach)
            % Checks for the valid combination of TimeIndex and
            % ConfigurationIndex (CI), as described in TS 38.211 Section
            % 5.3.2 and Tables 6.3.3.2-2 to 6.3.3.2-4.
            
            errorFlag = any(prach.TimeIndex >= prach.NumTimeOccasions);
            coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidTimeIndexForCI',sprintf('%d',int8(prach.TimeIndex)),sprintf('%d',int8(prach.NumTimeOccasions(1)-1)));
        end 
        
        function checkLRA(obj,val)
            % Checks for the validity of LRA, given format and subcarrier
            % spacing
            
            format = obj.Format;
            switch format
                case {'0','1','2','3'}
                    errorFlag = (val ~= 839);
                    coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidLRALong',sprintf('%g',double(val)),format);
                case {'A1','A2','A3','B1','B2','B3','B4','C0','C2'}
                    scs = obj.SubcarrierSpacing;
                    if scs == 15
                        errorFlag = ~any(val == [139, 1151]);
                        coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidLRASharedSpectrum',sprintf('%g',double(val)),'1151',format,sprintf('%g',double(scs)));
                    elseif any(scs == [30, 480])
                        errorFlag = ~any(val == [139, 571]);
                        coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidLRASharedSpectrum',sprintf('%g',double(val)),'571',format,sprintf('%g',double(scs)));
                    elseif any(scs == [60, 960])
                        errorFlag = val ~= 139;
                        coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidLRAShort',sprintf('%g',double(val)),format);
                    elseif scs == 120
                        errorFlag = ~any(val == [139, 571, 1151]);
                        coder.internal.errorIf(errorFlag,'nr5g:nrPRACHConfig:InvalidLRASharedSpectrum',sprintf('%g',double(val)),'571, 1151',format,sprintf('%g',double(scs)));
                    else % Wrong combination of format and subcarrier spacing
                        checkSCSAndFormat(obj,obj);
                    end
                otherwise % Wrong configuration
                    checkCIAndFRAndDuplexMode(obj,obj);
            end
        end
        
    end % Cross-parameter checks
    
    % Private methods
    methods (Access = private)
        function obj = setLRA(obj,val)
            % Private method to set LRA
            checkLRA(obj,val);
            obj.theLRA = val;
        end
        
        function out = getLRA(obj)
            % LRA is updated based on Format and SubcarrierSpacing:
            % TS 38.211 Tables 6.3.3.1-1 and 6.3.3.1-2
            
            out = obj;
            switch obj.Format
                case {'0','1','2','3'} % Long preamble
                    table = nr5g.internal.prach.getTable6331x(1);
                    temp = table.LRA(strcmpi(table.Format,obj.Format));
                otherwise % Short preamble
                    % For short preambles, LRA is set by default to 139.
                    % If the subcarrier spacing is 15, 30, 120, or 480 kHz
                    % and the previous value of LRA is not 139 or 839, the
                    % nrPRACHConfig object assumes a shared spectrum and
                    % sets the right value of LRA.
                    sharedSpectrum = any(obj.theLRA == [571, 1151]);
                    table = nr5g.internal.prach.getTable6331x(2);
                    scs = obj.SubcarrierSpacing;
                    if ~sharedSpectrum
                        temp = table.LRA(strcmpi(table.Format,obj.Format));
                    else
                        if any(scs==[60,960])
                            temp = table.LRA(strcmpi(table.Format,obj.Format));
                        elseif scs == 15
                            temp = table.LRA_03(strcmpi(table.Format,obj.Format));
                        elseif any(scs == [30,480])
                            temp = table.LRA_135(strcmpi(table.Format,obj.Format));
                        else
                            % For 120 kHz, all values of LRA are allowed.
                            % Use the last one stored in the theLRA
                            % property.
                            temp = obj.theLRA;
                        end
                    end
            end
            if ~isempty(temp)
                out.theLRA = temp(1);
            else
                out.theLRA = NaN;
            end
        end
    end
    
    % Protected methods to control the conditional display of properties
    methods (Access = protected)
        
        function flag = isInactiveProperty(obj, prop)
            flag = false;
            
            lra = obj.LRA;
            
            % FrequencyIndex only if LRA is one of [139, 839], or LRA is
            % one of [571, 1151] and FR2
            if strcmp(prop,'FrequencyIndex')
                flag = any(lra==[571, 1151]) && obj.FrequencyRange=="FR1";
            end
            
            % RBSetOffset only if LRA is one of [571, 1151] and FR2
            if strcmp(prop,'RBSetOffset')
                flag = ~(any(lra==[571, 1151]) && obj.FrequencyRange=="FR1");
            end
        end
        
    end % Protected methods
    
    % Static methods to define default inputs for string parameters
    methods(Hidden, Static, Access = private)
        
        function out = defaultRestrictedSet
            %defaultRestrictedSet Set the default value for the property RestrictedSet
            
            out = 'UnrestrictedSet';
        end
        
    end % Static methods
    
end