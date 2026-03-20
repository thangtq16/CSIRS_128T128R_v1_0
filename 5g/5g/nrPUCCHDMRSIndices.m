function ind = nrPUCCHDMRSIndices(carrier,pucch,varargin)
%nrPUCCHDMRSIndices Physical uplink control channel DM-RS resource element indices
%   IND = nrPUCCHDMRSIndices(CARRIER,PUCCH) returns a column vector IND
%   containing demodulation reference signal (DM-RS) resource element (RE)
%   indices of physical uplink control channel, as defined in TS 38.211
%   Section 6.4.1.3, for all physical uplink control channel formats.
%   CARRIER is a scalar nrCarrierConfig object. For physical uplink control
%   channel formats 0, 1, 2, 3, and 4, PUCCH is a scalar nrPUCCH0Config,
%   nrPUCCH1Config, nrPUCCH2Config, nrPUCCH3Config, and nrPUCCH4Config,
%   respectively. The output IND is empty for physical uplink control
%   channel format 0.
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
%   IND = nrPUCCHDMRSIndices(CARRIER,PUCCH,NAME,VALUE,...) specifies
%   additional options as NAME,VALUE pairs to allow control over the format
%   of the indices:
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
%   SecondHopStartPRB properties are ignored.
%
%   Example 1:
%   % Generate the DM-RS indices of a physical uplink control channel with
%   % format 1 occupying first resource block in the bandwidth part. The
%   % starting OFDM symbol and number of OFDM symbols allocated for PUCCH
%   % is 3 and 9, respectively. The bandwidth part occupies the complete
%   % 10 MHz bandwidth of a 15 kHz subcarrier spacing carrier.
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
%   % Get PUCCH format 1 DM-RS symbols and indices
%   dmrsSym = nrPUCCHDMRS(carrier,pucch1);
%   dmrsInd = nrPUCCHDMRSIndices(carrier,pucch1);
%
%   % Map the DM-RS in carrier grid and visualize the grid
%   resGrid = nrResourceGrid(carrier);
%   resGrid(dmrsInd) = dmrsSym;
%   imagesc(abs(resGrid))
%   axis xy
%   xlabel('OFDM Symbols')
%   ylabel('Subcarriers')
%   title('Carrier Grid Containing PUCCH DM-RS')
%
%   Example 2:
%   % Generate the DM-RS indices of a physical uplink control channel with
%   % format 3 occupying first 12 resource blocks in the bandwidth part.
%   % The starting OFDM symbol and number of OFDM symbols allocated for
%   % PUCCH is 3 and 9, respectively. Configure DM-RS with additional
%   % DM-RS. The bandwidth part occupies the complete 10 MHz bandwidth of a
%   % 15 kHz subcarrier spacing carrier.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSizeGrid = 52;
%
%   % Set PUCCH parameters
%   pucch3 = nrPUCCH3Config;
%   pucch3.NSizeBWP = [];
%   pucch3.NStartBWP = [];
%   pucch3.PRBSet = 0:11;
%   pucch3.SymbolAllocation = [3 9];
%   pucch3.AdditionalDMRS = 1;
%
%   % Get the PUCCH DM-RS symbols and indices
%   dmrsSym = nrPUCCHDMRS(carrier,pucch3);
%   dmrsInd = nrPUCCHDMRSIndices(carrier,pucch3);
%
%   % Map the DM-RS in carrier grid and visualize the grid
%   resGrid = nrResourceGrid(carrier);
%   resGrid(dmrsInd) = dmrsSym;
%   imagesc(abs(resGrid))
%   axis xy
%   xlabel('OFDM Symbols')
%   ylabel('Subcarriers')
%   title('Carrier Grid Containing PUCCH DM-RS')
%
%   See also nrPUCCHDMRS, nrPUCCHIndices, nrPUCCH1Config, nrPUCCH2Config,
%   nrPUCCH3Config, nrPUCCH4Config.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [formatPUCCH,~,freqHopping] = nr5g.internal.pucch.validateInputObjects(carrier,pucch);

    % Parse options
    fcnName = 'nrPUCCHDMRSIndices';
    opts = nr5g.internal.parseOptions(fcnName,...
        {'IndexStyle','IndexBase','IndexOrientation'},varargin{:});

    % Validate allocation
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,prbset,symbperslot] = ...
            nr5g.internal.pucch.validateAllocation(carrier,pucch);

    % Get PUCCH DM-RS indices
    if isempty(pucch.SymbolAllocation) || numel(prbset) == 0 || ...
            (pucch.SymbolAllocation(2) == 0) || (formatPUCCH == 0)
        % Return empty output with options
        if strcmpi(opts.IndexStyle,'index')
            ind = uint32(zeros(0,1));
        else
            ind = uint32(zeros(0,3));
        end
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
        nRBSC = 12;             % Number of subcarriers in a resource block
        scInd = (0:(nRBSC-1))'; % Subcarrier indices (0...11)

        % Get the OFDM symbols allocated for PUCCH
        symAllocation = double(pucch.SymbolAllocation);
        lastPUCCHSym = symAllocation(1) + symAllocation(2) - 1;

        % Get the subcarrier and OFDM symbol locations of PUCCH DM-RS
        switch formatPUCCH
            case 1
                % PUCCH format 1, TS 38.211 Section 6.4.1.3.1.2
                % DM-RS OFDM symbol locations
                ldmrs = symAllocation(1):2:lastPUCCHSym;
                % Get the number of OFDM symbols in each hop
                nSF = ceil(symAllocation(2)/2);
                if intraSlotFreqHopping
                    if rem(symAllocation(2),2) == 1
                        nSF0 = floor(nSF/2);
                    else
                        nSF0 = ceil(nSF/2);
                    end
                else
                    nSF0 = nSF;
                end
                nSF1 = nSF - nSF0;
                % DM-RS subcarrier locations
                firstHopK = nRBSC*prbsetHop(1,:) + scInd;
                secondHopK = nRBSC*prbsetHop(2,:) + scInd;
                Kdmrs = [repmat(firstHopK(:),1,nSF0) repmat(secondHopK(:),1,nSF1)];
            case 2
                % PUCCH format 2, TS 38.211 Section 6.4.1.3.2.2
                nSF1 = double(symAllocation(2) == 2);
                % DM-RS OFDM symbol locations
                ldmrs = symAllocation(1):lastPUCCHSym;
                % DM-RS subcarrier locations
                scInd = [1 4 7 10].';
                firstSymK = nRBSC*prbsetHop(1,:) + scInd;
                secondSymK = nRBSC*prbsetHop(2,:) + scInd;
                Kdmrs = [firstSymK(:) repmat(secondSymK(:),1,nSF1)];
            otherwise
                % PUCCH format 3 or 4, TS 38.211 Section 6.4.1.3.3.2
                % DM-RS OFDM symbol locations
                ldmrs = nr5g.internal.pucch.dmrsSymbolIndicesFormats34(...
                    symAllocation,intraSlotFreqHopping,pucch.AdditionalDMRS);
                numSym = numel(ldmrs);

                % DM-RS subcarrier locations
                firstHopK = nRBSC*prbsetHop(1,:) + scInd;
                if intraSlotFreqHopping
                    secondHopK = nRBSC*prbsetHop(2,:) + scInd;
                    Kdmrs = [repmat(firstHopK(:),1,numSym/2) ...
                             repmat(secondHopK(:),1,numSym/2)];
                else
                    Kdmrs = repmat(firstHopK(:),1,numSym);
                end
        end
        % Repeat the OFDM symbol locations to number of subcarriers in each
        % OFDM symbol
        Ldmrs = repmat(ldmrs,size(Kdmrs,1),1);

        % Apply options
        carrierRef = 0; % The indices passed to applyIndexOrientation are with respect to BWP
        ind = nr5g.internal.applyIndicesOptions([nSizeBWP*nRBSC symbperslot 1],...
            opts,Kdmrs(:),Ldmrs(:));
        ind = nr5g.internal.applyIndexOrientation([nStartGrid nSizeGrid],...
            [nStartBWP nSizeBWP],symbperslot,opts,ind,carrierRef);
    end

end
