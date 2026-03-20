function info = nrPRGInfo(carrier,prgsize)
%nrPRGInfo Precoding resource block group (PRG)-related information
%   INFO = nrPRGInfo(CARRIER,PRGSIZE) provides information related to
%   precoding resource block group (PRG) bundling, defined in TS 38.214
%   Section 5.1.2.3.
%
%   CARRIER is a carrier configuration object, <a
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these
%   object properties are relevant for this function:
%   NSizeGrid  - Number of resource blocks in carrier resource grid
%                (1...275)
%   NStartGrid - Start of carrier resource grid relative to CRB 0
%                (0...2199)
%
%   PRGSIZE is the PRG bundle size (2, 4, or [] to indicate 'wideband').
%
%   INFO is a structure containing the fields:
%   NPRG       - Number of PRGs in common resource blocks 0...NCRB-1
%   PRGSet     - Column vector of 1-based PRG indices for each RB in the 
%                carrier grid, size CARRIER.NSizeGrid-by-1
%
%   The values of NPRG corresponding to values of PRGSIZE are as follows:
%   PRGSIZE =  2: NPRG = ceil(NCRB / 2)
%   PRGSIZE =  4: NPRG = ceil(NCRB / 4)
%   PRGSIZE = []: NPRG = 1 ('wideband')
%   where NCRB is the number of common resource blocks (CRBs) between point
%   A and the last CRB of the carrier resource grid, inclusive. That is, 
%   NCRB = carrier.NStartGrid + carrier.NSizeGrid.
%
%   Example:
%   % Get the PRG information for a carrier configuration with 10 resource
%   % blocks and a PRG bundle size of 4. The carrier has 3 PRGs, with the
%   % first two PRGs containing 4 PRBs, and the last PRG containing 2 PRBs.
%
%   carrier = nrCarrierConfig;
%   carrier.NSizeGrid = 10;
%   prgSize = 4;
%   
%   prgInfo = nrPRGInfo(carrier,prgSize);
%
%   See also nrCarrierConfig, nrPDSCHPrecode.

%#codegen

% Copyright 2021-2024 The MathWorks, Inc.

    narginchk(2,2);

    % Validate inputs
    validateInputs(carrier,prgsize);

    % Calculate the number of carrier resource blocks (CRB) spanning the
    % carrier grid including the starting CRB offset
    NCRB = double(carrier.NStartGrid) + double(carrier.NSizeGrid);

    % Handle the case of empty PRG size, which configures a single fullband
    % PRG
    prgsize = double(prgsize);
    if (isempty(prgsize))
        Pd_BWP = NCRB;
    else
        Pd_BWP = prgsize;
    end

    % Calculate the number of precoding resource block groups
    NPRG = ceil(NCRB / Pd_BWP);

    % Calculate the 1-based PRG indices for each RB in the carrier grid
    prgset = ...
        nr5g.internal.prgSet(double(carrier.NSizeGrid),double(carrier.NStartGrid),NPRG);

    % Create the info output
    info.NPRG = NPRG;
    info.PRGSet = prgset;

end

% Validate inputs
function validateInputs(carrier,prgsize)

    fcnName = 'nrPRGInfo';

    % Validate carrier input
    validateattributes(carrier,{'nrCarrierConfig'},...
        {'scalar'},fcnName,'Carrier specific configuration object');

    % Validate prgsize
    if ~(isnumeric(prgsize) && isempty(prgsize))
        validateattributes(prgsize,{'numeric'}, ...
            {'scalar','integer'},fcnName,'PRG bundle size');
        coder.internal.errorIf(~any(prgsize(1)==[2 4]), ...
            'nr5g:nrPRGInfo:InvalidPRGSize',prgsize(1));
    end

end
