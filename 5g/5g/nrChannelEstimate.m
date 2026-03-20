function [H,nVar,info] = nrChannelEstimate(varargin)
%nrChannelEstimate Practical channel estimation
%   [H,NVAR,INFO] = nrChannelEstimate(...) performs channel estimation,
%   returning channel estimate H, noise variance estimate NVAR and
%   information structure INFO. H is a K-by-N-by-R-by-P array where K is
%   the number of subcarriers, N is the number of OFDM symbols, R is the
%   number of receive antennas and P is the number of reference signal
%   ports. NVAR is scalar indicating the measured variance of additive
%   white Gaussian noise on the received reference symbols. It can also be
%   a column vector indicating the measured noise variance on the received
%   reference symbols for each precoding resource block group (PRG) in the
%   case of PDSCH PRG bundling. INFO is a structure containing the field:
%   AveragingWindow - a 2-element row vector [F T] indicating the number 
%                     of adjacent reference symbols in the frequency
%                     direction F and time direction T over which averaging
%                     was performed prior to interpolation. In the case of
%                     PDSCH PRG bundling, this is a 2-column matrix.
%
%   [H,NVAR,INFO] = nrChannelEstimate(CARRIER,RXGRID,REFIND,REFSYM)
%   performs channel estimation on the received resource grid RXGRID using
%   reference symbols REFSYM whose locations are given by REFIND.
%
%   CARRIER is a carrier configuration object, <a 
%   href="matlab:help('nrCarrierConfig')"
%   >nrCarrierConfig</a>. Only these
%   object properties are relevant for this function:
%
%   CyclicPrefix      - Cyclic prefix ('normal', 'extended')
%   NStartGrid        - Start of the carrier resource grid relative to CRB
%                       0 (only relevant in the case of PDSCH PRG bundling)
%
%   RXGRID is an array of size K-by-L-by-R. K is the number of subcarriers,
%   given by CARRIER.NSizeGrid * 12. L is the number of OFDM symbols in one
%   slot, given by CARRIER.SymbolsPerSlot.
%
%   REFIND and REFSYM are the reference signal indices and symbols,
%   respectively. REFIND is an array of 1-based linear indices addressing a
%   K-by-L-by-P resource array. P is the number of reference signal ports
%   and is inferred from the range of values in REFIND. Only nonzero
%   elements in REFSYM are considered. Any zero-valued elements in REFSYM
%   and their associated indices in REFIND are ignored.
%
%   [H,NVAR,INFO] = nrChannelEstimate(CARRIER,RXGRID,REFGRID) specifies a
%   predefined reference resource grid in REFGRID. REFGRID is an array with
%   nonzero elements representing the reference symbols in their
%   appropriate locations. It is of size K-by-N-by-P, where N is the number
%   of OFDM symbols. REFGRID can span multiple slots. RXGRID must be an
%   array of size K-by-N-by-R.
%
%   [H,NVAR,INFO] =
%   nrChannelEstimate(CARRIER,RXGRID,REFIND,REFSYM,NAME,VALUE,...) and
%   [H,NVAR,INFO] =
%   nrChannelEstimate(CARRIER,RXGRID,REFGRID,NAME,VALUE,...) perform
%   channel estimate as above, but an additional NAME,VALUE pair is used to
%   specify the PRG bundle size:
%
%   'PRGBundleSize'   - The PDSCH PRG bundle size (2, 4, or [] (default),
%                       where [] indicates no PDSCH PRG bundling)
%
%   [H,NVAR,INFO] = nrChannelEstimate(RXGRID,REFIND,REFSYM,NAME,VALUE,...)
%   and [H,NVAR,INFO] = nrChannelEstimate(RXGRID,REFGRID,NAME,VALUE,...)
%   perform channel estimation as above, but an additional NAME,VALUE pair
%   is used in place of the CARRIER configuration object:
%
%   'CyclicPrefix'      - Cyclic prefix ('normal' (default), 'extended')
%
%   Note that for the numerologies specified in TS 38.211 Section 4.2, 
%   extended cyclic prefix length is only applicable for 60 kHz subcarrier
%   spacing. For normal cyclic prefix there are L=14 OFDM symbols in a 
%   slot. For extended cyclic prefix, L=12.
%
%   [H,NVAR,INFO] = nrChannelEstimate(...,NAME,VALUE,...) specifies
%   additional options as NAME,VALUE pairs:
%
%   'CDMLengths'      - A 2-element row vector [FD TD] specifying the 
%                       length of FD-CDM and TD-CDM despreading to perform.
%                       A value of 1 for an element indicates no CDM and a
%                       value greater than 1 indicates the length of the
%                       CDM. For example, [2 1] indicates FD-CDM2 and no
%                       TD-CDM. The default is [1 1] (no orthogonal
%                       despreading)
%
%   'AveragingWindow' - A 2-element row vector [F T] or a 2-column matrix
%                       whose rows are [F T] in the PDSCH PRG bundling case 
%                       specifying the number of adjacent reference symbols
%                       in the frequency domain F and time domain T over
%                       which to average prior to interpolation. F and T
%                       must be odd or zero. If F or T is zero, the
%                       averaging value is determined automatically from
%                       the estimated SNR (calculated using NVAR). If a row
%                       vector is provided in the PDSCH PRG bundling case,
%                       the specified averaging values will be applied to
%                       all PRGs.
%
%   'Interpolation'   - Toggle to control interpolation between
%                       reference symbols. When set to 'off',
%                       nrChannelEstimate does not interpolate between the
%                       specified reference symbols. The default value is
%                       'on'.
%
%   Example:
%   % Create a resource grid containing the PDSCH DM-RS and pass it through
%   % a TDL-C channel. Estimate the channel response and compare it with
%   % the perfect channel estimator.
%
%   carrier = nrCarrierConfig;
%   pdsch = nrPDSCHConfig;
%   dmrsInd = nrPDSCHDMRSIndices(carrier,pdsch);
%   dmrsSym = nrPDSCHDMRS(carrier,pdsch);
%   nTxAnts = 1;
%   txGrid = nrResourceGrid(carrier,nTxAnts);
%   txGrid(dmrsInd) = dmrsSym;
%
%   [txWaveform,ofdmInfo] = nrOFDMModulate(carrier,txGrid);
%
%   channel = nrTDLChannel;
%   channel.NumTransmitAntennas = nTxAnts;
%   channel.NumReceiveAntennas = 1;
%   channel.SampleRate = ofdmInfo.SampleRate;
%   channel.DelayProfile = 'TDL-C';
%   channel.DelaySpread = 100e-9;
%   channel.MaximumDopplerShift = 20;
%   chInfo = info(channel);
%   maxChDelay = ceil(max(chInfo.PathDelays*channel.SampleRate)) + chInfo.ChannelFilterDelay;
%   [rxWaveform,pathGains] = channel([txWaveform; zeros(maxChDelay,nTxAnts)]);
%   
%   offset = nrTimingEstimate(carrier,rxWaveform,dmrsInd,dmrsSym);
%   rxWaveform = rxWaveform(1+offset:end,:);
%
%   rxGrid = nrOFDMDemodulate(carrier,rxWaveform);
%
%   [H,nVar,estInfo] = nrChannelEstimate(carrier,rxGrid,dmrsInd,dmrsSym);
%
%   pathFilters = getPathFilters(channel);
%   H_ideal = nrPerfectChannelEstimate(carrier,pathGains,pathFilters,offset);
%
%   figure;
%   subplot(1,2,1);
%   imagesc(abs(H));
%   xlabel('OFDM symbol');
%   ylabel('Subcarrier');
%   title('Practical estimate magnitude');
%   subplot(1,2,2);
%   imagesc(abs(H_ideal));
%   xlabel('OFDM symbol');
%   ylabel('Subcarrier');
%   title('Perfect estimate magnitude');
%
%   See also nrTimingEstimate, nrPerfectChannelEstimate, 
%   nrPerfectTimingEstimate, nrCarrierConfig.

%   Copyright 2019-2024 The MathWorks, Inc.

%#codegen

    % ---------------------------------------------------------------------
    % Input validation and parsing, pre-processing
    % ---------------------------------------------------------------------

    narginchk(2,12);

    % Get optional inputs, inputs whose position depends upon the syntax,
    % or variables that depend upon optional inputs
    % rxGrid: received grid
    % refInd: reference signal indices
    % refSym: reference signal symbols
    % N: number of OFDM symbols
    % P: number of transmit antenna ports
    % ofdmInfo: OFDM dimensionality information
    % estConfig: channel estimator configuration
    % ECP: extended cyclic prefix
    % prgSize: PRG bundle size
    % prgInfo: PRG bundling information
    [rxGrid,refInd,refSym,N,P,ofdmInfo,estConfig,ECP,prgSize,prgInfo] = ...
        getOptionalInputs(varargin{:});

	coder.varsize('refInd',[Inf Inf],[1 1]);

    isGPU = false;
    if isa(rxGrid,"gpuArray") || isa(refInd,"gpuArray") || isa(refSym,"gpuArray")
        [rxGrid,refInd,refSym] = gather(rxGrid,refInd,refSym);
        isGPU = true;
    end

    % Get channel estimate output dimensions, final channel estimate will
    % be of size K-by-N-by-R-by-P
    K = ofdmInfo.NSubcarriers;
    R = size(rxGrid,3);

    % Get processing policy
    [policy,kpattern,nRefSymPRG,fracPRGInd,uprg,prgKranges] = ...
        nr5g.internal.nrChannelEstimate.getInterpolationPolicy(K,N,P, ...
        refInd,prgSize,prgInfo);
    NPRG = max(1,numel(uprg));

    % 'eK' is the number of extra subcarriers that will be added to the
    % channel estimate grid (half at each side), to mitigate band edge
    % effects when reconstructing the channel frequency response (CFR)
    % from the denoised channel impulse response (CIR)
    eRB = 4;
    eK = eRB * 12;

    % Create the channel estimate grid, including 'eK' extra subcarriers
    siz = [K+eK N R P];
    H = complex(zeros(siz,'like',rxGrid));

    % Create arrays 'kmin' and 'kmax' which will be used to store the 
    % minimum and maximum frequency subscripts, for each PRG in each OFDM
    % symbol and transmit port. For PRG that can be processed in parallel,
    % store the minimum and maximum frequency subscripts corresponding to
    % kpattern in 'kminpattern' and 'kmaxpattern' respectively
    kmin = zeros([N P NPRG]);
    kmax = zeros([N P NPRG]);
    kpatternmin = zeros([N,P]);
    kpatternmax = zeros([N,P]);

    % Create array 'refRBs' which will be used to identify the resource
    % blocks which contain reference symbols, for each OFDM symbol and
    % transmit port
    NRB = K / 12;
    refRBs = zeros([NRB+eRB N P]);

    % ---------------------------------------------------------------------
    % LS estimation, CDM despreading
    % ---------------------------------------------------------------------

    % Extract CDM despreading parameters
    fdCDM = estConfig.CDMLengths(1);
    tdCDM = estConfig.CDMLengths(2);

    % For each transmit port
    noPRGBundling = isempty(prgSize);
    for p = 1:P

        % Get frequency (subcarrier k) and time (OFDM symbol n) subscripts
        % of reference signal for the current port. 'thisport' is a logical
        % indexing vector for the current port, used to extract the
        % corresponding reference symbols
        [ksubs,nsubs,thisport] = getPortSubscripts(K,N,P,refInd,p);
        refSymThisPort = refSym(thisport);

        % For each OFDM symbol
        un = unique(nsubs).';
        for uni = 1:numel(un)

            % Get frequency and OFDM symbol subscripts
            n = un(uni);
            k = ksubs(nsubs==n);

            % Initialize frequency subscripts 'ke' to be used for
            % interpolation, taking extra subcarriers into account. Also
            % mark resource blocks (RBs) containing reference signals for
            % this OFDM symbol and port in 'refRBs'
            ke = k + eK/2;
            rbsubs = unique(floor((ke-1)/12));
            refRBs(rbsubs + 1,n,p) = 1;

            % Calculate 'kmin' and 'kmax', the minimum and maximum
            % frequency subscripts for PRGs in this symbol and port
            [kmin,kmax,kpatternmin,kpatternmax] = ...
                nr5g.internal.nrChannelEstimate.calculateKMinMax(kmin,kmax,kpatternmin,kpatternmax, ...
                p,n,k,eK,prgKranges,policy(p,n),fracPRGInd{p,n},kpattern{p,n});

            % For each receive antenna
            for r = 1:R

                % Perform least squares (LS) estimate of channel in the 
                % locations of the reference symbols. 'H_LS' is a column
                % vector containing the LS estimates for all subcarriers
                % for the current port, OFDM symbol and receive antenna
                H_LS = rxGrid(k,n,r) ./ refSymThisPort(nsubs==n);

                % Perform FD-CDM despreading if required
                if (fdCDM>1)

                    H_LS = ...
                        nr5g.internal.nrChannelEstimate.despreadFDCDM(H_LS, ...
                        fdCDM,noPRGBundling,nRefSymPRG{p,n});

                end

                % Assign the estimates into the appropriate region of
                % the overall channel estimate array
                H(ke,n,r,p) = H_LS;

            end

        end

        % Perform TD-CDM despreading if required
        if (tdCDM>1)

            H(:,:,:,p) = ...
                nr5g.internal.nrChannelEstimate.despreadTDCDM(H(:,:,:,p), ...
                un,tdCDM,K,eK,R);

        end

    end

    % ---------------------------------------------------------------------
    % Interpolation in frequency direction, CIR denoising
    % ---------------------------------------------------------------------

    % Calculate minimum cyclic prefix length in terms of a DFT of size K+eK
    cp = floor(min(ofdmInfo.CyclicPrefixLengths) / ofdmInfo.Nfft * (K+eK));

    % Create time-domain windowing function for CIR denoising
    w = raised_cosine_window(cp*2,cp);
    w = [w; zeros([K+eK-length(w) 1])];
    w = circshift(w,-cp-floor(cp/2));

    % Create matrix 'Hn' which will be used to store channel estimates used
    % for noise estimation
    Hn = NaN(size(H),'like',H);

    % Perform CIR denoising for each transmit port
    for p = 1:P

        [H,Hn] = ...
            nr5g.internal.nrChannelEstimate.denoiseCIR(p,K,N,P,R,eK,refInd, ...
            kmin,kmax,kpatternmin,kpatternmax,refRBs,H,Hn,fdCDM,w,policy, ...
            kpattern,prgKranges,fracPRGInd);

    end

    % Remove extra subcarriers from the channel estimate that were added
    % during interpolation, remove the corresponding resource blocks from
    % 'RBs' and adjust 'kmax' appropriately ('kmin' and 'kmax' are used
    % subsequently to determine indices for interpolation in the time
    % direction)
    H = H(eK/2 + (1:K),:,:,:);
    Hn = Hn(eK/2 + (1:K),:,:,:);
    refRBs = refRBs(eRB/2 + (1:NRB),:,:);
    kmax = kmax - eK;
    kpatternmax = kpatternmax - eK;

    % ---------------------------------------------------------------------
    % Noise estimation, averaging parameter selection
    % ---------------------------------------------------------------------

    [nVar,estConfig,info] = ...
        nr5g.internal.nrChannelEstimate.estimateNoise(H,Hn,R,ECP,fdCDM, ...
        tdCDM,refRBs,estConfig,prgKranges);

    % ---------------------------------------------------------------------
    % Averaging and interpolation in frequency direction
    % ---------------------------------------------------------------------

    % Extract frequency averaging parameter
    freqAveraging = estConfig.AveragingWindow(:,1);

    % For each transmit port
    for p = 1:P

        % Get time (OFDM symbol n) subscripts of reference signal for the
        % current port
        [~,nsubs] = getPortSubscripts(K,N,P,refInd,p);

        % For each OFDM symbol
        un = unique(nsubs).';
        for uni = 1:numel(un)

            % Get current OFDM symbol number 'n'
            n = un(uni);

            % For each receive antenna
            for r = 1:R

                H = ...
                    nr5g.internal.nrChannelEstimate.freqAverageAndInterpolate(H, ...
                    n,r,p,kmin,kmax,kpatternmin,kpatternmax,freqAveraging, ...
                    prgKranges,policy(p,n),fracPRGInd{p,n});

            end

        end

    end

    % For each PRG, each OFDM symbol for each port, blank any subcarriers
    % outside of the RBs that originally contained reference symbols
    for g = 1:numel(uprg)
        rbLow = floor((prgKranges(g,1)-1)/12);
        rbHigh = floor((prgKranges(g,2)-1)/12);
        for p = 1:P
            un = find((kmin(:,p,g).')~=0);
            for uni = 1:numel(un)

                n = un(uni);
                rbsubs = find(refRBs(:,n,p)==0) - 1;
                rbsubs = rbsubs(rbsubs>=rbLow & rbsubs<=rbHigh);
                if (~isempty(rbsubs))
                    ksubs = (rbsubs*12 + (1:12)).';
                    ksubs = ksubs(:);
                    H(ksubs,n,:,p) = 0;
                end

            end
        end
    end

    % ---------------------------------------------------------------------
    % Averaging and interpolation in time direction
    % ---------------------------------------------------------------------

    % Extract time averaging parameter
    timeAveraging = estConfig.AveragingWindow(:,2);

    % For each transmit port
    for p = 1:P

        H = ...
            nr5g.internal.nrChannelEstimate.timeAverageAndInterpolate(H, ...
            refRBs,p,NRB,R,tdCDM,timeAveraging,N,policy,fracPRGInd,prgKranges);

    end

    if ~estConfig.Interpolation & ~isempty(H)
        % Extract non-interpolated matrix
        [kAllP,nAllP,~] = ind2sub([K N P],refInd(:));
        un = unique(nAllP);
        Htemp = zeros(size(H),"like",H);
        for i = 1:numel(un)
            n = un(i);
            k = unique(kAllP(nAllP==n));
            Htemp(k,n,:,:) = H(k,n,:,:);
        end
        H = Htemp;
    end

    if isGPU
        H = gpuArray(H);
    end

end

%% Local functions

% Raised cosine window creation; creates a window function of length n+w
% with raised cosine transitions on the first and last 'w' samples.
function p = raised_cosine_window(n,w)

    p = 0.5*(1-sin(pi*(w+1-2*(1:w).')/(2*w)));
    p = [p; ones([n-w 1]); flipud(p)];

end

% Gets k,n subscripts for K-by-N-by-P grid given indices 'ind' and port
% 'port'. 'thisport' is a logical indexing vector for the port 'port', used
% to extract the corresponding reference symbols
function [ksubs,nsubs,thisport] = getPortSubscripts(K,N,P,ind,port)

    [ksubs,nsubs,psubs] = ind2sub([K N P],ind(:));

    thisport = (psubs==port);

    ksubs = ksubs(thisport);
    nsubs = nsubs(thisport);

end

% Parse optional inputs
function [rxGrid,refInd,refSym,N,P,ofdmInfo,cec,ECP,prgSize,prgInfo] = getOptionalInputs(varargin)

    fcnName = 'nrChannelEstimate';

    % Determine if syntax with nrCarrierConfig is being used and parse
    % relevant inputs
    isCarrierSyntax = isa(varargin{1},'nrCarrierConfig');
    if (isCarrierSyntax)
        carrier = varargin{1};
        validateattributes(carrier,{'nrCarrierConfig'}, ...
            {'scalar'},fcnName,'Carrier specific configuration object');
        rxGrid = varargin{2};
        firstrefarg = 3;
    else
        rxGrid = varargin{1};
        firstrefarg = 2;
    end

    % Validate type of grid and number of dimensions in grid, dimension
    % sizes are validated when other relevant parameters are known
    % (refInd,refSym versus refGrid syntax, normal versus extended cyclic
    % prefix)
    validateattributes(rxGrid,{'double','single'},{'3d'},fcnName,'RXGRID');
    K = size(rxGrid,1);
    coder.internal.errorIf(mod(K,12)~=0, ...
        'nr5g:nrChannelEstimate:InvalidRxGridSubcarriers',K);
    NRB = K / 12;
    if (~(isCarrierSyntax && carrier.NSizeGrid>275))
        coder.internal.errorIf(NRB>275, ...
            'nr5g:nrChannelEstimate:InvalidRxGridSubcarrierSize',K);
    end

    % Determine whether the refInd,refSym syntax or refGrid syntax is being
    % used
    isRefGridSyntax = ...
        (nargin==firstrefarg) || ischar(varargin{firstrefarg + 1}) ...
            || isstring(varargin{firstrefarg + 1});
    if (isRefGridSyntax)
        % nrChannelEstimate(...,refGrid,...)
        firstoptarg = firstrefarg + 1;
    else
        % nrChannelEstimate(...,refInd,refSym,...)
        firstoptarg = firstrefarg + 2;
    end

    % Parse options
    if (isCarrierSyntax)
        optNames = {'CDMLengths','AveragingWindow','PRGBundleSize','Interpolation'};
        opts = nr5g.internal.parseOptions( ...
            fcnName,optNames,varargin{firstoptarg:end});
    else
        optNames = {'CyclicPrefix','CDMLengths','AveragingWindow','Interpolation'};
        opts = nr5g.internal.parseOptions( ...
            fcnName,optNames,varargin{firstoptarg:end});
    end

    % Get OFDM information. Subcarrier spacing (SCS) is hard-wired to 15
    % kHz because SCS does not affect OFDM information fields as used in
    % the channel estimator:
    %  * SampleRate is not used
    %  * SymbolsPerSubframe is not used
    %  * larger CP lengths every 0.5ms are not used, because only
    %    min(CyclicPrefixLengths) is used
    %  * length of CyclicPrefixLengths is not used
    if (isCarrierSyntax)
        ECP = strcmpi(carrier.CyclicPrefix,'extended');
    else
        ECP = strcmpi(opts.CyclicPrefix,'extended');
    end
    SCS = 15;
    ofdmInfo = nr5g.internal.OFDMInfo(NRB,SCS,ECP,struct());

    % Get the number of subcarriers K and OFDM symbols L from the OFDM 
    % information
    K = ofdmInfo.NSubcarriers;
    L = ofdmInfo.SymbolsPerSlot;

    % Validate reference inputs
    if (isRefGridSyntax)

        refGrid = varargin{firstrefarg};

        % Validate reference grid
        validateattributes(refGrid, ...
            {'double','single'},{'finite','3d'},fcnName,'REFGRID');
        coder.internal.errorIf(size(refGrid,1)~=K, ...
            'nr5g:nrChannelEstimate:InvalidRefGridSubcarriers', ...
            size(refGrid,1),K);

        % Get the number of OFDM symbols 'N' in the reference grid
        N = size(refGrid,2);

        % Get number of ports 'P' in the reference grid
        P = size(refGrid,3);

    else

        refInd = varargin{firstrefarg};
        refSym = varargin{firstrefarg + 1};

        % Validate reference indices and place in a single column
        validateattributes(refInd,{'numeric'}, ...
            {'positive','finite','2d'},fcnName,'REFIND');
        refIndColumns = size(refInd,2);
        refInd = double(refInd(:));
        coder.internal.errorIf(numel(refInd)~=numel(unique(refInd)), ...
            'nr5g:nrChannelEstimate:NonUniqueIndices');

        % Validate reference symbols and place in a single column
        validateattributes(refSym,{'double','single'}, ...
            {'finite','2d'},fcnName,'REFSYM');
        if (any(refSym(:)==0))
            coder.internal.warning('nr5g:nrChannelEstimate:ZeroValuedSym');
        end
        refSymColumns = size(refSym,2);
        refSym = refSym(:);
        coder.internal.errorIf(numel(refSym)~=numel(refInd), ...
            'nr5g:nrChannelEstimate:UnequalSymIndCount', ...
            numel(refSym),numel(refInd));

        % The number of OFDM symbols 'N' in the implied reference grid is
        % 'L', the number of OFDM symbols in one slot
        N = L;

        % Get the number of ports, based on the range of the reference
        % symbol indices
        if (isempty(refInd) && isempty(refSym) && refIndColumns==refSymColumns)
            P = refIndColumns;
        else
            P = ceil(max(refInd/(K*L)));
            if (~isfinite(P))
                P = 1;
            end
        end

        % Validate received grid OFDM symbol dimension, it must span one 
        % slot
        coder.internal.errorIf(size(rxGrid,2)~=L, ...
            'nr5g:nrChannelEstimate:InvalidRxGridOFDMSymbols', ...
            size(rxGrid,2),L);

        % Create reference grid and map reference symbols
        refGrid = zeros([K L P],'like',refSym);
        refGrid(refInd) = refSym;

    end

    % Extract reference indices and symbols from reference grid
    refInd = find(refGrid(:)~=0);
    refSym = refGrid(refInd);

    % Validate reference grid OFDM symbol dimension, it must be equal to 
    % the number of OFDM symbols in the received grid
    coder.internal.errorIf(N~=size(rxGrid,2), ...
        'nr5g:nrChannelEstimate:InvalidRefGridOFDMSymbols', ...
        N,size(rxGrid,2));

    % Get CDM length from options
    cec.CDMLengths = double(opts.CDMLengths);
    cec.AveragingWindow = double(opts.AveragingWindow);
    coder.varsize('cec.AveragingWindow',[Inf,2],[1,0]);

    % Get PRG info
    if (isCarrierSyntax)
        % Calculate PRG info when non-empty PRG bundle size is provided in
        % the carrier syntax
        prgSize = opts.PRGBundleSize;
        prgInfo = nrPRGInfo(carrier,prgSize);
    else
        % In non-carrier syntax, PRG bundling is not supported; empty PRG
        % size provided in carrier syntax also suggests wideband precoding.
        % In these cases, directly construct the info structure to avoid
        % unnecessary validations and calculations.
        prgSize = [];
        prgInfo = struct('NPRG',1,'PRGSet',ones(NRB,1));
    end

    % Get interpolation option
    cec.Interpolation = strcmp(opts.Interpolation,'on');

end