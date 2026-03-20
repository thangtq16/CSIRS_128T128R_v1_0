function [waveform,info] = nrPRACHOFDMModulate(carrier,prach,grid,varargin)
%nrPRACHOFDMModulate PRACH OFDM modulation
%   [WAVEFORM,INFO] = nrPRACHOFDMModulate(CARRIER,PRACH,GRID) performs OFDM
%   modulation of a physical random access channel (PRACH) slot resource
%   array, GRID, given uplink carrier configuration object CARRIER and
%   PRACH configuration object PRACH.
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
%   GRID is a complex K-by-L-by-P array. P is the number of antennas. K is
%   the number of subcarriers, given by:
%   (CARRIER.SubcarrierSpacing/PRACH.SubcarrierSpacing)*CARRIER.NSizeGrid*12.
%   L is the number of OFDM symbols in the grid, given by:
%    - PRACH.PRACHDuration for long formats
%    - 7 for short preamble format C0
%    - 14 for all other short preamble formats
%
%   WAVEFORM is a T-by-P matrix where T is the number of time-domain 
%   samples in the PRACH waveform for the current slot PRACH.NPRACHSlot.
%   The value of T is:
%      INFO.OffsetLength + sum(INFO.SymbolLengths)
%   where INFO.SymbolLengths is:
%      INFO.CyclicPrefixLengths + INFO.Nfft + INFO.GuardLengths
%
%   Note that for long formats (PRACH.LRA=839), the first subframe of a
%   PRACH preamble can occur partway through the nominal PRACH slot period.
%   In this case, INFO.OffsetLength is increased to ensure that the OFDM
%   waveform will span the entire active PRACH preamble. To balance these
%   longer PRACH slots with the nominal PRACH slot period, some inactive
%   PRACH slots will have OFDM waveforms that are shorter than the nominal
%   PRACH slot period. This is conveyed in the OFDM information by
%   INFO.CyclicPrefixLengths and INFO.GuardLengths being empty (i.e. no
%   OFDM symbols) and INFO.OffsetLength will equal the number of empty
%   subframes required. In all cases, the expressions above for the number
%   of time-domain samples T are applicable. For more information, see
%   the examples for <a href="matlab:help('nrPRACHOFDMInfo')"
%   >nrPRACHOFDMInfo</a>.
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
%   [WAVEFORM,INFO] = nrPRACHOFDMModulate(...,NAME,VALUE) specifies
%   additional options as NAME,VALUE pairs to allow control over the OFDM
%   modulation:
%
%   Windowing           - Number of time-domain samples over which 
%                         windowing and overlapping of OFDM symbols is 
%                         applied. If absent or set to [], a default value
%                         is selected based on other parameters, see 
%                         <a href="matlab: doc('nrPRACHOFDMModulate')"
%                         >nrPRACHOFDMModulate</a> for details
%
%   % Example:
%
%   % Configure carrier
%   carrier = nrCarrierConfig;
%
%   % Configure PRACH for format A1
%   prach = nrPRACHConfig;
%   prach.ConfigurationIndex = 106;
%   prach.SubcarrierSpacing = 15;
%
%   % Create PRACH grid and map PRACH preamble to the grid
%   grid = nrPRACHGrid(carrier,prach);
%   ind = nrPRACHIndices(carrier,prach);
%   sym = nrPRACH(carrier,prach);
%   grid(ind) = sym;
%
%   % OFDM modulate the PRACH grid
%   [waveform,info] = nrPRACHOFDMModulate(carrier,prach,grid);
%
%   See also nrPRACHOFDMInfo, nrPRACHGrid, nrPRACH, nrPRACHIndices, 
%   nrPRACHConfig.

%   Copyright 2020-2022 The MathWorks, Inc.
    
%#codegen

    narginchk(3,5);

    % Validate inputs and get PRACH OFDM information
    [prach,internalinfo] = parseAndValidateInputs(carrier,prach,grid,varargin{:});

    % Get PRACH OFDM-modulated waveform
    waveform = nr5g.internal.prach.OFDMModulate(carrier,prach,grid,internalinfo);

    % Create OFDM information output structure
    info = nr5g.internal.prach.OFDMInfoOutput(internalinfo);

end

% Parse and validate inputs
function [prach,info] = parseAndValidateInputs(carrier,prach,grid,varargin)
    
    % Validate carrier and PRACH inputs
    fcnName = 'nrPRACHOFDMModulate';
    prach = nr5g.internal.prach.validatePRACHAndCarrier(carrier,prach,fcnName);

    % Parse options
    optNames = {'Windowing'};
    opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{:});
    
    % Get OFDM information
    info = nr5g.internal.prach.OFDMInfo(carrier,prach,opts);

    % Validate grid
    validateattributes(grid,{'double','single'},{'3d'},fcnName,'GRID');
    Kgrid = size(grid,1);
    Lgrid = size(grid,2);
    Kinfo = info.NSubcarriers;
    Linfo = numel(info.CyclicPrefixLengths);
    coder.internal.errorIf(Kgrid~=Kinfo,'nr5g:nrPRACHOFDMModulate:InvalidGridSubcarriers',Kgrid,Kinfo);
    if (Linfo~=0)
        coder.internal.errorIf(Lgrid~=Linfo,'nr5g:nrPRACHOFDMModulate:InvalidGridOFDMSymbols',Lgrid,Linfo);
    end

end
