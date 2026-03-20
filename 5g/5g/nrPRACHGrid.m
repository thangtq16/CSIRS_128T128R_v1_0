function grid = nrPRACHGrid(carrier,prach,varargin)
%nrPRACHGrid Physical random access channel slot resource grid
%   GRID = nrPRACHGrid(CARRIER,PRACH) returns an empty physical random
%   access channel (PRACH) slot resource grid, as a complex array of all
%   zeros, for one antenna and the specified carrier and PRACH
%   configurations.
%
%   GRID = nrPRACHGrid(CARRIER,PRACH,P) also specifies the number of
%   antennas, P.
%
%   GRID is a complex K-by-L-by-P array of all zeros. P is the number of
%   antennas. K is the number of subcarriers, given by:
%   (carrier.SubcarrierSpacing/prach.SubcarrierSpacing)*carrier.NSizeGrid*12.
%   L is the number of OFDM symbols in the grid, given by:
%    - PRACH.PRACHDuration for long formats
%    - 7 for short preamble format C0
%    - 14 for all other short preamble formats
%
%   CARRIER is a carrier configuration object, <a
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these
%   object properties are relevant for this function:
%
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15, 30, 60, 120, 240, 480, 960)
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%
%   PRACH is a PRACH configuration object, <a
%   href="matlab:help('nrPRACHConfig')"
%   >nrPRACHConfig</a>. Only these
%   object properties are relevant for this function:
%
%   FrequencyRange      - Frequency range (used in combination with
%                         DuplexMode to select a configuration table from
%                         TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%                         ('FR1', 'FR2')
%   DuplexMode          - Duplex mode (used in combination with
%                         FrequencyRange to select a configuration table
%                         from TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%                         ('FDD', 'SUL', 'TDD')
%   ConfigurationIndex  - Configuration index, as defined in TS 38.211
%                         Tables 6.3.3.2-2 to 6.3.3.2-4 (0...262)
%   SubcarrierSpacing   - PRACH subcarrier spacing in kHz
%                         (1.25, 5, 15, 30, 60, 120, 480, 960)
%
%   GRID = nrPRACHGrid(...,NAME,VALUE) specifies additional options as
%   NAME,VALUE pairs to allow control over the data type of the output
%   grid:
%
%   'OutputDataType'       - 'double' for double precision (default)
%                            'single' for single precision
%
%   Example:
%   % Create an empty PRACH slot resource grid for PRACH configuration
%   % index 107 (preamble format A1) and 30 kHz PUSCH subcarrier spacing:
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 30;
%
%   prach = nrPRACHConfig;
%   prach.ConfigurationIndex = 107;
%   prach.SubcarrierSpacing = 15;
%
%   grid = nrPRACHGrid(carrier,prach);
%
%   See also nrCarrierConfig, nrPRACHConfig.

%   Copyright 2019-2022 The MathWorks, Inc.

    %#codegen
    narginchk(2,5);
    
    % Input validation
    [prach,P,opts] = parseAndValidateInputs(carrier,prach,varargin{:});

    % Compute the grid size
    NRB = carrier.NSizeGrid;
    SCS = carrier.SubcarrierSpacing;
    gridSize = nr5g.internal.prach.gridSize(NRB,SCS,prach,P);
    
    % Generate an empty grid
    grid = complex(zeros(gridSize,opts.OutputDataType));

end

% Parse and validate inputs
function [prach,P,opts] = parseAndValidateInputs(carrier,prach,varargin)
    
    % Validate carrier and PRACH inputs
    fcnName = 'nrPRACHGrid';
    prach = nr5g.internal.prach.validatePRACHAndCarrier(carrier,prach,fcnName);

    % Validate and parse the number of antennas
    if ((nargin>2 && isnumeric(varargin{1})) || nargin==3)
        P = varargin{1};
        validateattributes(P, {'numeric'}, {'nonempty','scalar','positive','integer'}, fcnName, 'Number of antennas');
        firstoptarg = 2;
    else
        P = 1;
        firstoptarg = 1;
    end

    % Parse options
    optNames = {'OutputDataType'};
    opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{firstoptarg:end});

end
