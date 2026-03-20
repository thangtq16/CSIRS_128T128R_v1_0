function ind = nrPUSCHDMRSIndices(carrier,pusch,varargin)
%nrPUSCHDMRSIndices Physical uplink shared channel DM-RS resource element indices
%   IND = nrPUSCHDMRSIndices(CARRIER,PUSCH) returns the matrix IND
%   containing 1-based demodulation reference signal (DM-RS) resource
%   element (RE) indices of physical uplink shared channel within the
%   carrier resource grid, in linear form. The matrix IND is obtained from
%   TS 38.211 Section 6.4.1.1.3, for the given carrier configuration
%   CARRIER, and uplink shared channel configuration PUSCH. The number of
%   columns in IND is equal to the number of antenna ports configured.
%
%   CARRIER is a carrier configuration object, as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with properties:
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
%   PUSCH is the physical uplink shared channel configuration object, as
%   described in <a href="matlab:help('nrPUSCHConfig')">nrPUSCHConfig</a> with properties:
%
%   NSizeBWP           - Size of the bandwidth part (BWP) in terms
%                        of number of physical resource blocks (PRBs)
%                        (1...275) (default []). The default value implies
%                        the value is equal to the size of carrier resource
%                        grid
%   NStartBWP          - Starting PRB index of BWP relative to CRB 0
%                        (0...2473) (default []). The default value implies
%                        the value is equal to the start of carrier
%                        resource grid
%   NumLayers          - Number of transmission layers (1...4) (default 1)
%   MappingType        - Mapping type of physical uplink shared channel
%                        ('A' (default), 'B')
%   SymbolAllocation   - Symbol allocation of physical uplink shared
%                        channel (default [0 14]). This property is a
%                        two-element vector. First element represents the
%                        start of OFDM symbol in a slot. Second element
%                        represents the number of contiguous OFDM symbols
%   PRBSet             - PRBs allocated for physical uplink shared channel
%                        within a BWP (0-based) (default 0:51)
%   TransformPrecoding - Flag to enable transform precoding
%                        (0 (default), 1). 0 indicates that transform
%                        precoding is disabled and the waveform type is
%                        CP-OFDM. 1 indicates that transform precoding is
%                        enabled and waveform type is DFT-s-OFDM
%   TransmissionScheme - Transmission scheme of physical uplink shared
%                        channel ('nonCodebook' (default), 'codebook')
%   NumAntennaPorts    - Number of antenna ports (1 (default), 2, 4)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - PRB start for second hop relative to the BWP
%                        (0-based) (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   DMRS               - PUSCH-specific DM-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHDMRSConfig')">nrPUSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType  - DM-RS configuration type (1 (default), 2).
%                                When transform precoding is enabled, the
%                                value must be 1
%       DMRSTypeAPosition      - Position of first DM-RS OFDM symbol in a
%                                slot (2 (default), 3)
%       DMRSLength             - Number of consecutive DM-RS OFDM symbols
%                                (1 (default), 2). When intra-slot
%                                frequency hopping is enabled, the value
%                                must be 1. Value of 1 indicates
%                                single-symbol DM-RS. Value of 2 indicates
%                                double-symbol DM-RS
%       DMRSAdditionalPosition - Maximum number of DM-RS additional
%                                positions (0...3) (default 0). When
%                                intra-slot frequency hopping is enabled,
%                                the value must be either 0 or 1
%       DMRSPortSet            - DM-RS antenna port set (0...11)
%                                (default []). The default value implies
%                                that the values are in the range from 0 to
%                                NumLayers-1
%       CustomSymbolSet        - Custom DM-RS symbol locations (0-based)
%                                (default []). This property is used to
%                                override the standard defined DM-RS symbol
%                                locations. Each entry corresponds to a
%                                single-symbol DM-RS
%
%   IND = nrPUSCHDMRSIndices(CARRIER,PUSCH,NAME,VALUE,...) specifies
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
%   For operation with shared spectrum channel access for FR1, set
%   Interlacing = true and specify the allocated frequency resources using
%   the RBSetIndex and InterlaceIndex properties of the PUSCH
%   configuration. The PRBSet, FrequencyHopping, and SecondHopStartPRB
%   properties are ignored.
%
%   Example 1:
%   % Generate the DM-RS indices (0-based) in linear index form of a
%   % physical uplink shared channel occupying the 10 MHz bandwidth for a
%   % 15 kHz subcarrier spacing (SCS) carrier. Configure DM-RS with
%   % type A position set to 2, configuration type set to 1, number of
%   % additional positions set to 0, and length set to 1.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pusch = nrPUSCHConfig;
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   ind = nrPUSCHDMRSIndices(carrier,pusch,'IndexBase','0based');
%
%   Example 2:
%   % Generate DM-RS indices of a physical uplink shared channel with
%   % transform precoding enabled, transmission scheme set to codebook,
%   % and number of antenna ports set to 4.
%
%   carrier = nrCarrierConfig;
%   pusch = nrPUSCHConfig('TransformPrecoding',1);
%   pusch.TransmissionScheme = 'codebook';
%   pusch.NumAntennaPorts = 4;
%   ind = nrPUSCHDMRSIndices(carrier,pusch);
%
%   Example 3:
%   % Generate and plot the DM-RS indices of a physical uplink shared
%   % channel occupying first 5 resource blocks of a 30 kHz SCS carrier
%   % having 5 MHz transmission bandwidth. Enable intra-slot frequency
%   % hopping with starting resource block of second hop at 4. Configure
%   % DM-RS with number of additional positions set to 1.
%
%   % Configure carrier with 30 kHz subcarrier spacing and 5 MHz bandwidth
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 30;
%   carrier.NSizeGrid = 11; % 5 MHz bandwidth
%
%   % Configure PUSCH
%   pusch = nrPUSCHConfig;
%   pusch.PRBSet = 0:4;
%   pusch.FrequencyHopping = 'intraSlot';
%   pusch.SecondHopStartPRB = 4;
%   pusch.DMRS.DMRSAdditionalPosition = 1;
%
%   % Get the resource element indices
%   ind = nrPUSCHDMRSIndices(carrier,pusch);
%
%   % Plot the indices on the grid
%   grid = zeros([carrier.NSizeGrid*12 carrier.SymbolsPerSlot pusch.NumLayers]);
%   grid(ind) = 1;
%   imagesc(abs(grid(:,:,1)));
%   axis xy;
%   xlabel('OFDM symbols');
%   ylabel('Subcarriers');
%   title('PUSCH DM-RS resource elements in the carrier resource grid');
%
%   See also nrPUSCHDMRS, nrTimingEstimate, nrChannelEstimate,
%   nrPUSCHPTRSIndices, nrPUSCHIndices, nrPUSCHConfig, nrPUSCHDMRSConfig,
%   nrCarrierConfig, nrIntraCellGuardBandsConfig.

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,8);

    % Validate inputs
    [nSizeGrid,nStartGrid,nSizeBWP,nStartBWP,symbperslot,freqHopping] = nr5g.internal.pusch.validateInputs(carrier,pusch);

    % Assign the structure ftable to pass into the initializeResources
    % internal function
    ftable.ChannelName = 'PUSCH';
    ftable.MappingTypeB = strcmpi(pusch.MappingType,'B');
    ftable.DMRSSymbolSet = @nr5g.internal.pusch.lookupPUSCHDMRSSymbols;
    ftable.IntraSlotFreqHoppingFlag = strcmpi(freqHopping,'intraSlot');

    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    % Get prbset, symbolset and dmrssymbols
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pusch,nSizeBWP,ftable);
    nsymbols = numel(symbolset);
    nRBSC = 12;

    % Assign the PRB set for each OFDM symbol in a cell array based on the
    % frequency hopping configuration
    prbcell = cell(1,symbperslot);
    for i = 1:symbperslot
        prbcell{i} = zeros(1,0);
    end
    if nsymbols
        startSym = min(symbolset);
    else
        startSym = 0;
    end
    prbsetHop = nr5g.internal.prbSetTwoHops(...
        prbset,freqHopping,pusch.SecondHopStartPRB,nslot);
    if strcmpi(freqHopping,'intraSlot')
        fhoplen = fix(nsymbols/2);
        for i = 1:fhoplen
            prbcell{i+startSym} = prbsetHop(1,:);
        end
        for i = fhoplen+1:nsymbols
            prbcell{i+startSym} = prbsetHop(2,:);
        end
    else % 'interSlot' or 'neither'
        for i = 1:nsymbols
            prbcell{i+startSym} = prbsetHop(1,:);
        end
    end

    % Capture the number of layers
    nLayers = double(pusch.NumLayers);

    % Get the number of antenna ports
    codebookTxFlag = strcmpi(pusch.TransmissionScheme,'codebook');
    if codebookTxFlag
        nports = double(pusch.NumAntennaPorts);
    else
        nports = nLayers;
    end

    % DM-RS subcarrier locations in a resource block for each port
    dmrsSubcarrierLocations = pusch.DMRS.DMRSSubcarrierLocations(:,1:nLayers);

    % Number of DM-RS resource elements in a resource block
    ndmrssc = size(dmrsSubcarrierLocations,1);

    % Get the DM-RS symbol indices
    nDMRSSymbols = numel(dmrssymbols);
    indPerLayer = zeros(0,nLayers);
    if nDMRSSymbols
        % Expand the subcarrier locations across all the active resource
        % blocks in a slot for all the ports
        nTotalRE = nRBSC*symbperslot*nSizeGrid; % Number of resource elements in a carrier grid of one layer/port
        for i = 1:nDMRSSymbols
            dmSymI = dmrssymbols(i);
            prbActive = reshape(repmat(nRBSC*(prbcell{dmSymI+1}+nStartBWP-nStartGrid),ndmrssc,1),[],1);
            indPerLayer = [indPerLayer; repmat(dmrsSubcarrierLocations,numel(prbcell{dmSymI+1}),1)...
                + repmat(prbActive,1,nLayers) + nRBSC*nSizeGrid*dmSymI + repmat(nTotalRE*(0:nLayers-1),numel(prbActive),1)]; %#ok<AGROW>
        end
    end

    % Codebook based transmission
    if codebookTxFlag && ~isempty(indPerLayer)
        % Use the CDM group number to label the DM-RS indices and group
        % them into different sets of rows. Within a PRB, we know that the
        % lower CDM groups have lower delta shifts
        cdm = zeros(3,1);
        cdm(pusch.DMRS.CDMGroups+1) = 1;
        cdmGroups = find(cdm)-1;
        ngroups = numel(cdmGroups);
        logicalMatrix = repmat(pusch.DMRS.CDMGroups,ngroups,1) == repmat(cdmGroups,1,nLayers);
        [cdmgroupsidx, ~] = find(logicalMatrix);
        cdmgroupsidx = reshape(cdmgroupsidx,1,[]);
        nDMRSPerLayer = numel(prbset)*ndmrssc*nDMRSSymbols;
        indices = repmat((cdmgroupsidx-1),nDMRSPerLayer,1) + reshape(1:ngroups:ngroups*nDMRSPerLayer*nLayers,[],nLayers);

        % DM-RS indices
        % Merge the indices used by the CDM groups
        % Each row is only associated with a single CDM
        pdmrsIndices = zeros(1,ngroups*nDMRSPerLayer);
        nreperslot = 12*symbperslot*nSizeGrid;
        normed = indPerLayer-repmat(((0:nLayers-1)*nreperslot),nDMRSPerLayer,1);
        pdmrsIndices(mod(indices(:)-1,ngroups*nDMRSPerLayer)+1) = normed;

        % Expand over all ports
        planeoffsets = nreperslot*(0:nports-1);
        ind = repmat(planeoffsets,numel(pdmrsIndices),1) + repmat(pdmrsIndices',[1 nports]);
    else
        ind = indPerLayer;
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPUSCHDMRSIndices';
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase','IndexOrientation','MultiColumnIndex'},varargin{:},'MultiColumnIndex',true);

        % Apply PV pairs 
        ind = nr5g.internal.applyIndicesOptions([nSizeGrid*nRBSC symbperslot nports],opts,ind(:));
        ind = nr5g.internal.applyIndexOrientation([nStartGrid nSizeGrid], [nStartBWP nSizeBWP],symbperslot,opts,ind);
    else
        % 1 based, linear indexing
        ind = uint32(ind + 1);
    end
end