function grid = nrResourceGrid(carrier,varargin)
%nrResourceGrid Carrier slot resource grid
%   GRID = nrResourceGrid(CARRIER) returns an empty carrier slot resource
%   grid, as a complex array of all zeros, for one antenna and the
%   specified carrier configuration.
%
%   GRID = nrResourceGrid(CARRIER,P) also specifies the number of antennas,
%   P.
%
%   CARRIER is a carrier configuration object, <a
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these 
%   object properties are relevant for this function:
%
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource grid
%                       (1...275)
%
%   GRID is a complex K-by-L-by-P array of zeros, where K is the number of
%   subcarriers, L is the number of OFDM symbols and P is the number of
%   antennas.
%
%   GRID = nrResourceGrid(...,NAME,VALUE) specifies additional options as
%   NAME,VALUE pairs to allow control over the data type of the output
%   grid:
%
%   'OutputDataType'       - 'double' for double precision (default)
%                            'single' for single precision
%
%   % Example:
%   % Create a carrier resource grid for 20 MHz bandwidth and 8 antennas
%
%   % Configure carrier for 20 MHz bandwidth
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 106;
%
%   % Create carrier resource grid for 8 antennas
%   grid = nrResourceGrid(carrier,8);
%   size(grid)
%
%   See also nrCarrierConfig, nrOFDMModulate, nrOFDMInfo, nrOFDMDemodulate.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

    narginchk(1,4);
    
    % Validate inputs
    [P,opts] = validateInputs(carrier,varargin{:});
    
    % Create resource grid
    K = double(carrier.NSizeGrid) * 12;
    L = carrier.SymbolsPerSlot;
    grid = complex(zeros([K L P],opts.OutputDataType));

end

% Get optional inputs
function [P,opts] = validateInputs(carrier,varargin)

    fcnName = 'nrResourceGrid';
    
    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},fcnName,'Carrier specific configuration object');
    
    if (nargin>1 && isnumeric(varargin{1}))
        P = varargin{1};
        validateattributes(P,{'numeric'},{'nonempty','scalar','positive','integer'},fcnName,'Number of antennas');
        firstoptarg = 2;
    else
        P = 1;
        firstoptarg = 1;
    end
    
    optNames = {'OutputDataType'};
    opts = nr5g.internal.parseOptions(fcnName,optNames,varargin{firstoptarg:end});
    
end
