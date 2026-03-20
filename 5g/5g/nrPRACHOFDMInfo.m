function info = nrPRACHOFDMInfo(carrier,prach,varargin)
%nrPRACHOFDMInfo PRACH OFDM modulation related information
%   INFO = nrPRACHOFDMInfo(CARRIER,PRACH) provides dimensional information
%   related to physical random access channel (PRACH) OFDM modulation,
%   given uplink carrier configuration object CARRIER and PRACH
%   configuration object PRACH.
%
%   CARRIER is a carrier configuration object, <a
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these 
%   object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15, 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%
%   PRACH is a PRACH configuration object, <a
%   href="matlab:help('nrPRACHConfig')"
%   >nrPRACHConfig</a>. Only these
%   object properties are relevant for this function:
%
%   FrequencyRange       - Frequency range (used in combination with
%                          DuplexMode to select a configuration table from
%                          TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%                          ('FR1', 'FR2')
%   DuplexMode           - Duplex mode (used in combination with
%                          FrequencyRange to select a configuration table
%                          from TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%                          ('FDD', 'SUL', 'TDD')
%   ConfigurationIndex   - Configuration index, as defined in TS 38.211
%                          Tables 6.3.3.2-2 to 6.3.3.2-4 (0...262)
%   SubcarrierSpacing    - PRACH subcarrier spacing in kHz
%                          (1.25, 5, 15, 30, 60, 120, 480, 960)
%   LRA                  - Length of the Zadoff-Chu preamble sequence
%                          (139, 571, 839, 1151)
%   NPRACHSlot           - PRACH slot number
%
%   INFO is a structure containing the fields:
%
%   Nfft                - Number of IFFT points used in the OFDM modulator
%   SampleRate          - Sample rate of the OFDM modulator
%   CyclicPrefixLengths - Cyclic prefix length (in samples) of each OFDM 
%                         symbol in the PRACH slot
%   GuardLengths        - Guard length (in samples) of each OFDM symbol in
%                         the PRACH slot
%   SymbolLengths       - Total length (in samples) of each OFDM symbol in
%                         the PRACH slot, including the cyclic prefix, 
%                         guard and offset
%   OffsetLength        - Length (in samples) of the initial time offset
%                         from the start of the configured PRACH slot 
%                         period to the start of the cyclic prefix
%   Windowing           - Number of time-domain samples over which 
%                         windowing and overlapping of OFDM symbols is 
%                         applied
%
%   Note that for long formats (PRACH.LRA=839), the first subframe of a
%   PRACH preamble can occur partway through the nominal PRACH slot period.
%   In this case, INFO.OffsetLength is increased to ensure that the OFDM
%   waveform produced by <a href="matlab:help('nrPRACHOFDMModulate')"
%   >nrPRACHOFDMModulate</a> will span the entire active
%   PRACH preamble. To balance these longer PRACH slots with the nominal
%   PRACH slot period, some inactive PRACH slots will have OFDM waveforms
%   that are shorter than the nominal PRACH slot period. This is conveyed
%   by INFO.CyclicPrefixLengths and INFO.GuardLengths being empty (i.e. no
%   OFDM symbols) and INFO.OffsetLength will equal the number of empty
%   subframes required. Note that PRACH.SubframesPerPRACHSlot is equal to
%   the nominal PRACH slot duration in all cases, whereas the actual length
%   in samples of a particular PRACH slot is given by:
%   INFO.OffsetLength + sum(INFO.SymbolLengths)
%
%   [WAVEFORM,INFO] = nrPRACHOFDMInfo(...,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the OFDM modulation:
%
%   Windowing           - Number of time-domain samples over which 
%                         windowing and overlapping of OFDM symbols is 
%                         applied. If absent or set to [], a default value
%                         is selected based on other parameters, see 
%                         <a href="matlab: doc('nrPRACHOFDMModulate')"
%                         >nrPRACHOFDMModulate</a> for details
%
%   % Example 1:
%   % Generate PRACH OFDM information for preamble format A1.
%
%   % Configure carrier
%   carrier = nrCarrierConfig;
%
%   % Configure PRACH for format A1
%   prach = nrPRACHConfig;
%   prach.ConfigurationIndex = 106;
%   prach.SubcarrierSpacing = 15;
%
%   % Create PRACH OFDM information
%   info = nrPRACHOFDMInfo(carrier,prach)
%
%   % Example 2:
%   % Generate PRACH OFDM information for multiple PRACH slots and show
%   % how the actual PRACH slot duration deviates from the nominal duration
%   % to ensure that the slot will always span entire active PRACH 
%   % preambles.
%
%   % Create carrier and PRACH configurations, calculate OFDM information
%   % and observe the nominal PRACH slot duration given by 
%   % SubframesPerPRACHSlot:
%
%   carrier = nrCarrierConfig;
%   prach = nrPRACHConfig;
%   prach.DuplexMode = 'FDD';
%   configurationIndex = 47; % Format 1
%   prach.ConfigurationIndex = configurationIndex;
%   ofdmInfo = nrPRACHOFDMInfo(carrier,prach);
%   totSubframes = prach.SubframesPerPRACHSlot
% 
%   % Use configuration tables to establish starting subframes for active
%   % PRACH preambles:
%
%   configTable = prach.Tables.ConfigurationsFR1PairedSUL;
%   subframeNumber = configTable.SubframeNumber{configurationIndex+1,:}
%
%   % The first nominal PRACH slot period, lasting totSubframes = 3 
%   % subframes, will span subframes 0, 1, and 2. However, Table 6.3.3.2-2
%   % of TS 38.211 specifies that the first active PRACH preamble should
%   % start in subframe 1 and end in subframe 3. Therefore the PRACH slot
%   % must contain an additional subframe to ensure that the PRACH slot
%   % spans the whole active preamble. To reflect this information,
%   % nrPRACHOFDMInfo sets the OffsetLength field to 1 subframe:
%
%   ofdmInfo.OffsetLength
%
%   % For NPRACHSlot = 1, the output of nrPRACHOFDMInfo shows that the 
%   % lengths of the symbol, cyclic prefix, and guard period are empty and
%   % the offset length is zero. Therefore the corresponding waveform will 
%   % last zero subframes. This is to balance longer PRACH slots (such as
%   % for NPRACHSlot = 0 above) with the nominal PRACH slot period:
%
%   prach.NPRACHSlot = 1;
%   ofdmInfo = nrPRACHOFDMInfo(carrier,prach)
% 
%   % Note that the next slot (NPRACHSlot = 2) will have a value of 
%   % OffsetLength equal to two subframes. The first subframe of the PRACH 
%   % slot is subframe 4 and without the additional subframes, it would 
%   % span subframes 4, 5 and 6. However, a PRACH preamble should start in 
%   % subframe 6 and occupy subframes 6, 7 and 8, so two extra subframes 
%   % are required:
%
%   prach.NPRACHSlot = 2;
%   ofdmInfo = nrPRACHOFDMInfo(carrier,prach)
%
%   See also nrPRACHOFDMModulate, nrPRACHGrid, nrPRACH, nrPRACHIndices, 
%   nrPRACHConfig.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,4);
    
    % Validate inputs and get OFDM information
    internalinfo = validateInputs(carrier,prach,varargin{:});
    
    % Create output structure
    info = nr5g.internal.prach.OFDMInfoOutput(internalinfo);
    
end

% Validate inputs
function info = validateInputs(carrier,prach,varargin)
    
    % Validate carrier and PRACH inputs
    fcnName = 'nrPRACHOFDMInfo';
    prach = nr5g.internal.prach.validatePRACHAndCarrier(carrier,prach,fcnName);
    
    % Parse options
    optNames = {'Windowing'};
    opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{:});
    
    % Get OFDM information
    info = nr5g.internal.prach.OFDMInfo(carrier,prach,opts);
    
end
