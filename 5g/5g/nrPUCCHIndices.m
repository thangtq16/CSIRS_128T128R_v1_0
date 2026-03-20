function [ind,info] = nrPUCCHIndices(carrier,pucch,varargin)
%nrPUCCHIndices Physical uplink control channel resource element indices
%   [IND,INFO] = nrPUCCHIndices(CARRIER,PUCCH) returns the column vector
%   IND containing 1-based physical uplink control channel resource element
%   (RE) indices within the carrier resource grid, in linear form. The
%   output IND is obtained from TS 38.211 Section 6.3.2, for all physical
%   uplink control channel formats. CARRIER is a scalar nrCarrierConfig
%   object. For physical uplink control channel formats 0, 1, 2, 3, and 4,
%   PUCCH is a scalar nrPUCCH0Config, nrPUCCH1Config, nrPUCCH2Config,
%   nrPUCCH3Config, and nrPUCCH4Config, respectively. This function also
%   provides the bit capacity and symbol capacity of uplink control
%   information (UCI) that is transmitted on physical uplink control
%   channel.
%
%   CARRIER is a carrier configuration object, as described in <a
%   href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>.
%   Only these object properties are relevant for this function:
%
%   SubcarrierSpacing   - Subcarrier spacing in kHz
%                         (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix        - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid           - Number of resource blocks in carrier resource
%                         grid (1...275) (default 52)
%   NStartGrid          - Start of carrier resource grid relative to common
%                         resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot               - Slot number (default 0)
%   IntraCellGuardBands - Intracell guard bands (default [])
%
%   For format 0, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH0Config')">nrPUCCH0Config</a>. Only these
%   object properties are relevant for this function:
%
%   NSizeBWP           - Size of bandwidth part (BWP) in terms of
%                        number of physical resource blocks (PRBs)
%                        (1...275) (default [])
%   NStartBWP          - Starting PRB index of BWP relative to common
%                        resource block 0 (CRB 0) (0...2473) (default [])
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [13 1])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - Starting PRB of second hop relative to the
%                        BWP (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%
%   For format 1, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH1Config')">nrPUCCH1Config</a>. Only these
%   object properties are relevant for this function:
%
%   NSizeBWP           - Size of bandwidth part (BWP) in terms of
%                        number of physical resource blocks (PRBs)
%                        (1...275) (default [])
%   NStartBWP          - Starting PRB index of BWP relative to common
%                        resource block 0 (CRB 0) (0...2473) (default [])
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - Starting PRB of second hop relative to the
%                        BWP (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%
%   For format 2, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH2Config')">nrPUCCH2Config</a>. Only these
%   object properties are relevant for this function:
%
%   NSizeBWP           - Size of bandwidth part (BWP) in terms of
%                        number of physical resource blocks (PRBs)
%                        (1...275) (default [])
%   NStartBWP          - Starting PRB index of BWP relative to common
%                        resource block 0 (CRB 0) (0...2473) (default [])
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [13 1])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - Starting PRB of second hop relative to the
%                        BWP (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   SpreadingFactor    - Spreading factor (2 (default), 4)
%
%   For format 3, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH3Config')">nrPUCCH3Config</a>. Only these
%   object properties are relevant for this function:
%
%   NSizeBWP           - Size of bandwidth part (BWP) in terms of
%                        number of physical resource blocks (PRBs)
%                        (1...275) (default [])
%   NStartBWP          - Starting PRB index of BWP relative to common
%                        resource block 0 (CRB 0) (0...2473) (default [])
%   Modulation         - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - Starting PRB of second hop relative to the
%                        BWP (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   SpreadingFactor    - Spreading factor (1, 2 (default), 4)
%   AdditionalDMRS     - Additional DM-RS configuration flag (0 (default), 1)
%
%   For format 4, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH4Config')">nrPUCCH4Config</a>. Only these
%   object properties are relevant for this function:
%
%   NSizeBWP           - Size of bandwidth part (BWP) in terms of
%                        number of physical resource blocks (PRBs)
%                        (1...275) (default [])
%   NStartBWP          - Starting PRB index of BWP relative to common
%                        resource block 0 (CRB 0) (0...2473) (default [])
%   Modulation         - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - Starting PRB of second hop relative to the
%                        BWP (0...274) (default 1)
%   AdditionalDMRS     - Additional DM-RS configuration flag (0 (default), 1)
%
%   The output structure INFO contains the following fields:
%   G             - Bit capacity of PUCCH. This value must be the length of
%                   UCI encoded codeword for formats 2, 3, and 4
%   Gd            - Symbol capacity of PUCCH
%   NREPerPRB     - Number of RE per PRB allocated to PUCCH (including
%                   spreading factor)
%   DMRSSymbolSet - The OFDM symbol locations in a slot containing DM-RS
%                   (0-based)
%   PRBSet        - PRBs allocated for PUCCH within the BWP
%
%   IND = nrPUCCHIndices(CARRIER,PUCCH,NAME,VALUE,...) specifies additional
%   options as NAME,VALUE pairs to allow control over the format of the
%   indices:
%
%    'IndexStyle'       - 'index' for linear indices (default)
%                         'subscript' for [subcarrier, symbol, antenna]
%                         subscript row form
%
%    'IndexBase'        - '1based' for 1-based indices (default)
%                         '0based' for 0-based indices
%
%    'IndexOrientation' - 'carrier' for carrier oriented indices (default)
%                         'bwp' for bandwidth part oriented indices
%
%   For PUCCH formats 0 to 3 and operation with shared spectrum channel
%   access for FR1, set Interlacing = true and specify the allocated
%   frequency resources using the RBSetIndex and InterlaceIndex properties
%   of the PUCCH configuration. The PRBSet, FrequencyHopping, and
%   SecondHopStartPRB properties are ignored. For PUCCH formats 2 and 3,
%   you can specify the SpreadingFactor for single-interlace
%   configurations.
%
%   Example 1:
%   % Generate the indices of physical uplink control channel with format 1
%   % occupying first resource block in the bandwidth part. The starting
%   % OFDM symbol and number of OFDM symbols allocated for PUCCH is 3 and 9,
%   % respectively. The bandwidth part occupies the complete 10 MHz
%   % bandwidth of a 15 kHz subcarrier spacing carrier.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSizeGrid = 52;
%
%   % Set PUCCH format 1 parameters
%   pucch1 = nrPUCCH1Config;
%   pucch1.NSizeBWP = [];
%   pucch1.NStartBWP = [];
%   pucch1.PRBSet = 0;
%   pucch1.SymbolAllocation = [3 9];
%
%   % Get PUCCH format 1 resource element indices
%   ind = nrPUCCHIndices(carrier,pucch1);
%
%   % Plot the resource elements in the carrier resource grid
%   resGrid = nrResourceGrid(carrier);
%   resGrid(ind) = 1;
%   imagesc(resGrid)
%   axis xy
%   xlabel('OFDM Symbols')
%   ylabel('Subcarriers')
%   title('Carrier Grid Containing PUCCH')
%
%   Example 2:
%   % Generate the indices of UCI modulated symbols of a physical uplink
%   % control channel with format 3 occupying first 12 resource blocks in
%   % the bandwidth part. The starting OFDM symbol and number of OFDM
%   % symbols allocated for PUCCH is 3 and 9, respectively. Configure DM-RS
%   % with additional DM-RS. The bandwidth part occupies the complete
%   % 10 MHz bandwidth of 15 kHz subcarrier spacing carrier.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSizeGrid = 52;
%
%   % Set PUCCH format 3 parameters
%   pucch3 = nrPUCCH3Config;
%   pucch3.NSizeBWP = [];
%   pucch3.NStartBWP = [];
%   pucch3.PRBSet = 0:11;
%   pucch3.SymbolAllocation = [3 9];
%   pucch3.AdditionalDMRS = 1;
%
%   % Get PUCCH format 3 resource element indices
%   ind = nrPUCCHIndices(carrier,pucch3);
%
%   % Plot the resource elements in the carrier resource grid
%   resGrid = nrResourceGrid(carrier);
%   resGrid(ind) = 1;
%   imagesc(resGrid)
%   axis xy
%   xlabel('OFDM Symbols')
%   ylabel('Subcarriers')
%   title('Carrier Grid Containing PUCCH')
%
%   See also nrPUCCH, nrPUCCH0, nrPUCCH1, nrPUCCH2, nrPUCCH3, nrPUCCH4,
%   nrPUCCHDMRSIndices, nrPUCCH0Config, nrPUCCH1Config, nrPUCCH2Config,
%   nrPUCCH3Config, nrPUCCH4Config, nrCarrierConfig,
%   nrIntraCellGuardBandsConfig.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [formatPUCCH,~,freqHopping] = nr5g.internal.pucch.validateInputObjects(carrier,pucch);
    
    % Validate allocation
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,prbset,symbperslot] = ...
        nr5g.internal.pucch.validateAllocation(carrier,pucch);

    % Initialize parameters
    nRBSC = 12;
    qm = 1;

    % Get the subcarrier and OFDM symbol locations of PUCCH
    nRE = 12; % Number of resource elements in a resource block used for UCI
    if isempty(pucch.SymbolAllocation) || (pucch.SymbolAllocation(2) == 0) || numel(prbset) == 0
        K = zeros(0,1);
        L = zeros(0,1);
        lsym = zeros(0,1);
        ldmrs = zeros(0,1);
    else
        % Relative slot number
        nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

        % Get the set of physical resource blocks for each hop depending on
        % frequency hopping configuration
        prbsetHop = nr5g.internal.prbSetTwoHops(...
            prbset,freqHopping,pucch.SecondHopStartPRB,nslot);

        % Initialize parameters related to intra-slot frequency hopping,
        % number of physical resource blocks allocated, and subcarrier
        % indices in a resource block
        intraSlotFreqHopping = strcmpi(freqHopping,'intraSlot');
        scInd = (0:(nRBSC-1))'; % Subcarrier indices (0...11)

        % Get the OFDM symbols allocated for PUCCH
        symAllocation = double(pucch.SymbolAllocation);
        lastPUCCHSym = symAllocation(1) + symAllocation(2) - 1;
        lsym = symAllocation(1):lastPUCCHSym;

        % Get the subcarrier and OFDM symbol locations where UCI
        % information is transmitted
        switch formatPUCCH
            case 0
                % PUCCH format 0, TS 38.211 Section 6.3.2.3.2
                % Subcarrier locations
                nSF1 = double(symAllocation(2) == 2);
                firstSymK = nRBSC*prbsetHop(1,:) + scInd;
                secondSymK = nRBSC*prbsetHop(2,:) + scInd;
                K = [firstSymK(:) repmat(secondSymK(:),1,nSF1)];
                % DM-RS OFDM symbol locations
                ldmrs = zeros(1,0);
            case 1
                % PUCCH format 1, TS 38.211 Section 6.3.2.4.2
                lTemp = lsym;
                lsym = lsym(2:2:end);
                numSymUCI = numel(lsym); % Number of OFDM symbols excluding DM-RS
                % Number of OFDM symbols in each hop
                if intraSlotFreqHopping
                    nSF0 = floor(symAllocation(2)/4);
                else
                    nSF0 = numSymUCI; % Equals to floor(symAllocation(2)/2)
                end
                nSF1 = numSymUCI - nSF0;
                % Subcarrier locations
                firstHopK = nRBSC*prbsetHop(1,:) + scInd;
                secondHopK = nRBSC*prbsetHop(2,:) + scInd;
                K = [repmat(firstHopK(:),1,nSF0) repmat(secondHopK(:),1,nSF1)];
                % DM-RS OFDM symbol locations
                ldmrs = lTemp(1:2:end);
            case 2
                % PUCCH format 2, TS 38.211 Section 6.3.2.5.3
                qm = 2; % Modulation is always QPSK
                nRE = 8; % Only 8 resource elements in a resource block are used for UCI
                % Subcarrier locations
                scInd = [0 2 3 5 6 8 9 11]'; % 0-based subcarrier indices excluding DM-RS (TS 38.211 Section 6.4.1.3.2.2)
                nSF1 = double(symAllocation(2) == 2);
                firstSymK = nRBSC*prbsetHop(1,:) + scInd;
                secondSymK = nRBSC*prbsetHop(2,:) + scInd;
                K = [firstSymK(:) repmat(secondSymK(:),1,nSF1)];
                % DM-RS OFDM symbol locations
                ldmrs = lsym;
            otherwise
                % PUCCH format 3 or 4, TS 38.211 Section 6.3.2.6.5
                qm = nr5g.internal.getQm(pucch.Modulation);
                lTemp = zeros(1,symbperslot);
                lTemp((symAllocation(1)+1):(lastPUCCHSym+1)) = 1;
                % DM-RS OFDM symbol locations
                ldmrs = nr5g.internal.pucch.dmrsSymbolIndicesFormats34(...
                    symAllocation,intraSlotFreqHopping,pucch.AdditionalDMRS);
                lTemp(ldmrs+1) = 2;
                % OFDM symbol locations removing DM-RS OFDM symbol
                % locations
                lsym = find(lTemp == 1) - 1;
                numSym = numel(lsym);
                % Subcarrier locations
                firstHopK = nRBSC*prbsetHop(1,:) + scInd;
                if intraSlotFreqHopping
                    secondHopK = nRBSC*prbsetHop(2,:) + scInd;
                    % Repeat subcarrier locations for each OFDM symbol in
                    % each hop
                    K = [repmat(firstHopK(:),1,floor(numSym/2)) ...
                             repmat(secondHopK(:),1,ceil(numSym/2))];
                else% No intra-slot frequency hopping
                    % Repeat subcarrier locations for each OFDM symbol
                    K = repmat(firstHopK(:),1,numSym);
                end
        end
        % Repeat the OFDM symbol locations to number of subcarriers in each
        % OFDM symbol
        L = repmat(lsym,size(K,1),1);
    end

    % Symbol and bit capacity
    [Gd,G] = channelCapacity(pucch,formatPUCCH,qm,numel(K));

    % Combine the information
    info = struct;
    info.G = G;
    info.Gd = Gd;
    info.NREPerPRB = nRE*length(lsym);
    info.DMRSSymbolSet = ldmrs;
    info.PRBSet = prbset(:).';

    % Apply options
    fcnName = 'nrPUCCHIndices';
    if nargin > 2
        opts = nr5g.internal.parseOptions(fcnName,...
            {'IndexStyle','IndexBase','IndexOrientation'},varargin{:});
    else
        opts = struct;
        opts.IndexStyle = 'index';
        opts.IndexBase = '1based';
        opts.IndexOrientation = 'carrier';
    end
    carrierRef = 0; % The indices passed to applyIndexOrientation are with respect to BWP
    ind = nr5g.internal.applyIndicesOptions([nSizeBWP*nRBSC symbperslot 1],...
        opts,K(:),L(:));
    ind = nr5g.internal.applyIndexOrientation([nStartGrid nSizeGrid],...
        [nStartBWP nSizeBWP],symbperslot,opts,ind,carrierRef);

end

% Calculate the symbol and bit capacity
function [Gd,G] = channelCapacity(pucch,formatPUCCH,qm,numRE)

    % Spreading factor can be configured for format 4, and formats 2
    % and 3 with a single interlace index. Spreading reduces the
    % capacity of the channel.
    sf = nr5g.internal.pucch.occConfiguration(pucch,formatPUCCH);
    if ~isempty(sf)
        gScaling = sf(1);
    else
        gScaling = 1;
    end

    Gd = numRE/gScaling(1);
    G = Gd*qm;

end
