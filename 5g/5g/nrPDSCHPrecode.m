function [antsym,antind] = nrPDSCHPrecode(carrier,portsym,portind,W)
%nrPDSCHPrecode Precoding for PDSCH PRG bundling
%   [ANTSYM,ANTIND] = nrPDSCHPrecode(CARRIER,PORTSYM,PORTIND,W) performs
%   the precoding for the PDSCH precoding resource block group (PRG)
%   bundling, as defined in TS 38.214 Section 5.1.2.3.
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
%   PORTSYM is a matrix of symbols to be precoded of size NRE-by-NLAYERS,
%   where NLAYERS is the number of layers.
% 
%   PORTIND is a matrix of the same size as PORTSYM, NRE-by-NLAYERS,
%   containing the 1-based linear indices of the symbols in PORTSYM. The
%   indices address a K-by-L-by-NLAYERS resource array. K is the number of
%   subcarriers, equal to CARRIER.NSizeGrid * 12. L is the number of OFDM
%   symbols in one slot, equal to CARRIER.SymbolsPerSlot. The precoding
%   performed by this function assumes that TS 38.211 Section 7.3.1.4 maps
%   layers to ports, that is, layers 0...NLAYERS-1 correspond to ports
%   0...NLAYERS-1.
%
%   W is an array of size NLAYERS-by-P-by-NPRG, where NPRG is the number of
%   PRGs in the carrier resource grid (see <a 
%   href="matlab:help('nrPRGInfo')">nrPRGInfo</a>). W defines a separate
%   precoding matrix of size NLAYERS-by-P for each PRG. Note that W must
%   contain precoding matrices for all PRGs between point A and the last
%   CRB of the carrier resource grid, inclusive.
%
%   ANTSYM is a matrix containing precoded PDSCH symbols. ANTSYM is of
%   size NRE-by-P, where NRE is number of PDSCH resource elements, and P is
%   the number of transmit antennas. 
%
%   ANTIND is a matrix containing the PDSCH antenna indices corresponding
%   to ANTSYM and is also of size NRE-by-P.
%
%   Optionally, PORTSYM and PORTIND can be of size NRE-by-R-by-P, where R
%   is the number of receive antennas. In this case, PORTSYM and PORTIND
%   define the symbols and indices of a PDSCH channel estimate. W must be
%   of size P-by-NLAYERS-by-NPRG. The channel estimate is precoded using
%   the P-by-NLAYERS matrices for each PRG bundle (the transpose of the
%   transmit precoding matrices). The outputs ANTSYM and ANTIND are of size
%   NRE-by-R-by-NLAYERS and provide the "effective channel" between receive
%   antennas and transmit layers. You can use this option to apply
%   precoding to a PDSCH allocation that you extract from the
%   antenna-oriented channel estimate returned by the
%   <a href="matlab:help('nrPerfectChannelEstimate')"
%   >nrPerfectChannelEstimate</a> function.
%   
%   Example 1:
%   % Perform PDSCH precoding using a PRG bundle size of 4 PRBs.
%   
%   % Configuration
%   carrier = nrCarrierConfig;
%   pdsch = nrPDSCHConfig;
%   prgsize = 4;
%   prginfo = nrPRGInfo(carrier,prgsize);
%
%   % Create PDSCH symbols
%   [portind,indinfo] = nrPDSCHIndices(carrier,pdsch);
%   cw = randi([0 1],indinfo.G,1);
%   portsym = nrPDSCH(carrier,pdsch,cw);
%
%   % Create random precoding matrix of correct size
%   nlayers = pdsch.NumLayers;
%   P = 4;
%   NPRG = prginfo.NPRG;
%   W = complex(randn([nlayers P NPRG]),randn([nlayers P NPRG]));
%
%   % Perform PDSCH precoding
%   [antsym,antind] = nrPDSCHPrecode(carrier,portsym,portind,W);
%
%   Example 2:
%   % Apply PDSCH precoding to a perfect channel estimate, to create the
%   % "effective channel" between receive antennas and transmit layers.
%
%   % Configuration
%   carrier = nrCarrierConfig;
%   ofdminfo = nrOFDMInfo(carrier);
%   pdsch = nrPDSCHConfig;
%   tdl = nrTDLChannel;
%   P = 3;
%   tdl.NumTransmitAntennas = P;
%   tdl.ChannelFiltering = false;
%   tdl.SampleRate = ofdminfo.SampleRate;
%   tdl.NumTimeSamples = tdl.SampleRate * 1e-3;
%   prgsize = 2;
%   prginfo = nrPRGInfo(carrier,prgsize);
%   portind = nrPDSCHIndices(carrier,pdsch);
%
%   % Create random precoding matrix of correct size
%   nlayers = pdsch.NumLayers;
%   NPRG = prginfo.NPRG;
%   W = complex(randn([nlayers P NPRG]),randn([nlayers P NPRG]));
%
%   % Channel
%   [pathGains,sampleTimes] = tdl();
%   pathFilters = getPathFilters(tdl);
%
%   % Perform channel estimation, extract PDSCH resources and apply
%   % precoding
%   H = nrPerfectChannelEstimate(carrier,pathGains,pathFilters);
%   [symH,indH] = nrExtractResources(portind,H);
%   pdschH = nrPDSCHPrecode(carrier,symH,indH,permute(W,[2 1 3]));
%
%   See also nrPRGInfo, nrPDSCH, nrPDSCHIndices, nrPerfectChannelEstimate.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(4,4);

    % Validate inputs
    portind = validateInputs(carrier,portsym,portind,W);

    % Calculate resource grid dimensions
    K = carrier.NSizeGrid * 12;
    L = carrier.SymbolsPerSlot;
    siz = [K L size(portind,2)];
    if (size(portind,3) > 1)
        siz = [siz size(portind,3)];
    end

    % Calculate 1-based PRG subscripts from RE subscripts and update 'siz'
    % in cases where it is missing a trailing singleton dimension
    [prgsubs,~,siz,allplanes] = ...
        nr5g.internal.prgSubscripts(siz,carrier.NStartGrid,portind,W);

    % Cross-validate sizes of symin, indin, and W
    validateSizes(siz,portsym,portind,W);

    % Perform precoding to produce antenna symbols and antenna indices
    [antsym,antind] = ...
        nr5g.internal.precode(siz,portsym,portind,W,prgsubs,allplanes);

end

% Validate inputs
function portind = validateInputs(carrier,portsym,portind,W)

    fcnName = 'nrPDSCHPrecode';

    % Validate carrier input
    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},...
        fcnName,'Carrier specific configuration object');

    % Validate portsym
    validateattributes(portsym,{'double','single'},{'finite','3d'}, ...
        fcnName,'PDSCH symbols');

    % Validate portind and convert to double
    validateattributes(portind,{'numeric'}, ...
        {'real','positive','finite','3d'},fcnName,'PDSCH indices');
    portind = double(portind);

    % Validate W
    validateattributes(W,{'double','single'},{'finite','3d'},fcnName,'W');

end

% Cross-validate sizes of portsym, portind, and W
function validateSizes(siz,portsym,portind,W)

    % Validate that portsym and portind are the same size
    symsiz = size(portsym,1:3);
    indsiz = size(portind,1:3);
    if (~isequal(symsiz,indsiz))
        v = num2cell([symsiz indsiz]);
        coder.internal.error('nr5g:nrPDSCHPrecode:UnequalSymIndSize',v{:});
    end

    % Validate that the first dimension size of W equals the last dimension
    % size of portind, unless the indices have been specified in a single
    % column (which allows for different numbers of REs per layer)
    if (size(portind,2)~=1)
        firstWdim = size(W,1);
        ndims = max(length(siz),3);
        lastinddim = indsiz(ndims-1);
        coder.internal.errorIf(firstWdim~=lastinddim, ...
            'nr5g:nrPDSCHPrecode:InvalidPrecoderSize',...
            firstWdim,lastinddim);
    end

end
