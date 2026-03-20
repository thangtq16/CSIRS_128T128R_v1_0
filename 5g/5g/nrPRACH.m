function [sym,info] = nrPRACH(carrier,prach,varargin)
%nrPRACH Physical random access channel
%   [SYM,INFO] = nrPRACH(CARRIER,PRACH) returns physical random access
%   channel (PRACH) symbols, SYM, as defined in TS 38.211 Section 6.3.3,
%   for the specified carrier and PRACH configurations. SYM is a complex
%   column vector. SYM is empty if the current PRACH preamble is not active
%   in the current subframe or 60 kHz slot, as described in TS 38.211
%   Section 6.3.3.2. The function also returns additional information,
%   INFO, as a structure with these fields:
%
%   RootSequence    - Physical root Zadoff-Chu sequence index or indices
%   CyclicShift     - Cyclic shift or shifts of Zadoff-Chu sequence
%   CyclicOffset    - For restricted set mode, cyclic shift or shifts
%                     corresponding to a Doppler shift of 1/T_SEQ
%   NumCyclicShifts - Number of cyclic shifts which corresponds to a single
%                     PRACH preamble sequence
%
%   CARRIER is a carrier configuration object, <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>.
%   Only these object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15, 30, 60, 120, 240, 480, 960)
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%
%   PRACH is a PRACH configuration object, <a href="matlab:help('nrPRACHConfig')">nrPRACHConfig</a>.
%   Only these object properties are relevant for this function:
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
%   SequenceIndex        - Logical root sequence index (0...1149)
%   PreambleIndex        - Scalar preamble index within cell (0...63)
%   RestrictedSet        - Type of restricted set ('UnrestrictedSet',
%                          'RestrictedSetTypeA', 'RestrictedSetTypeB')
%   ZeroCorrelationZone  - Cyclic shift configuration index (0...15)
%   ActivePRACHSlot      - Active PRACH slot number within a subframe or a
%                          60 kHz slot (0, 1, 3, 7, 15)
%   NPRACHSlot           - PRACH slot number
%
%   [SYM,INFO] = nrPRACH(...,NAME,VALUE) specifies additional options
%   as NAME,VALUE pairs to allow control over the data type of the output
%   symbols:
%
%   'OutputDataType'       - 'double' for double precision (default)
%                            'single' for single precision
%
%   If the value of PRACH.PreambleIndex is such that an insufficient
%   quantity of cyclic shifts are available at the configured logical root
%   index PRACH.SequenceIndex, the function increments the logical root
%   index number. As such, the physical root used, INFO.RootSequence,
%   differs from the physical root configured by PRACH.SequenceIndex. The
%   INFO.CyclicShift field represents the cyclic shift corresponding to
%   PRACH.PreambleIndex. Similarly, for high speed mode (i.e., restricted
%   set type A or B), if no valid cyclic shift exists for the current value
%   of PRACH.SequenceIndex, the function increments the logical root index
%   number.
%
%   Example:
%   % Generate PRACH symbols for the default configurations of
%   % nrCarrierConfig and nrPRACHConfig.
%
%   prach = nrPRACHConfig;
%   carrier = nrCarrierConfig;
%   [sym,info] = nrPRACH(carrier,prach);
%
%   See also nrCarrierConfig, nrPRACHConfig, nrPRACHIndices.

%   Copyright 2019-2022 The MathWorks, Inc.

    %#codegen
    narginchk(2,4);
    
    % Input validation
    [prach,opts] = parseAndValidateInputs(carrier,prach,varargin{:});
    
    % Get PRACH symbols and additional info
    info = nr5g.internal.prach.getSymbolsInfo(prach);
    sym = nr5g.internal.prach.getSymbols(prach,info,opts);

end

% Parse and validate inputs
function [prach,opts] = parseAndValidateInputs(carrier,prach,varargin)
    
    % Validate carrier and PRACH inputs
    fcnName = 'nrPRACH';
    prach = nr5g.internal.prach.validatePRACHAndCarrier(carrier,prach,fcnName);

    % Parse options
    optNames = {'OutputDataType'};
    opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{:});

end
