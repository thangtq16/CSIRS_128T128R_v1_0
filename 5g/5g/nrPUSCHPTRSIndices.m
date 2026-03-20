function ind = nrPUSCHPTRSIndices(carrier,pusch,varargin)
%nrPUSCHPTRSIndices Physical uplink shared channel PT-RS resource element indices
%   IND = nrPUSCHPTRSIndices(CARRIER,PUSCH) returns the matrix IND
%   containing 1-based phase tracking reference signal (PT-RS) resource
%   element (RE) indices of physical uplink shared channel within the
%   carrier resource grid, in linear form. The matrix IND is obtained from
%   TS 38.211 Section 6.4.1.2.2, for the given carrier configuration
%   CARRIER and uplink shared channel configuration PUSCH. Note that when
%   transform precoding is enabled, the indices are generated relative to
%   the start of the physical uplink shared channel allocation. The number
%   of columns in IND depends on the transmission scheme and transform
%   precoding. The number of columns in IND equals to:
%   - number of PT-RS antenna ports configured, when transform precoding is
%     disabled and transmission scheme is set to non-codebook
%   - number of antenna ports configured, when transform precoding is
%     disabled and transmission scheme is set to codebook
%   - number of transmission layers, when transform precoding is enabled
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
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
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
%   EnablePTRS         - Enable or disable the PT-RS configuration
%                        (0 (default), 1)
%   PTRS               - PUSCH-specific PT-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHPTRSConfig')">nrPUSCHPTRSConfig</a> with properties:
%       TimeDensity            - PT-RS time density (1 (default), 2, 4)
%    These properties are applicable, when transform precoding is set to 0:
%       FrequencyDensity       - PT-RS frequency density (2 (default), 4)
%       REOffset               - Resource element offset
%                                ('00' (default), '01', '10', '11')
%       PTRSPortSet            - PT-RS antenna port set (default []). The
%                                default value implies the value is equal
%                                to the lowest DM-RS antenna port
%                                configured
%    These properties are applicable, when transform precoding is set to 1:
%       NumPTRSSamples         - Number of PT-RS samples (2 (default), 4)
%       NumPTRSGroups          - Number of PT-RS groups (2 (default), 4, 8)
%
%   IND = nrPUSCHPTRSIndices(CARRIER,PUSCH,NAME,VALUE,...) specifies
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
%   % Generate PT-RS indices of a physical uplink shared channel occupying
%   % the 10 MHz bandwidth for a 15 kHz subcarrier spacing (SCS) carrier,
%   % with transform precoding set to 0. Configure DM-RS with number of
%   % additional positions set to 0, length set to 1, type A position set
%   % to 2, and configuration type set to 1. Enable PT-RS with time density
%   % set to 1, frequency density set to 2, and resource element offset set
%   % to '01'.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pusch = nrPUSCHConfig('TransformPrecoding',0);
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.TimeDensity = 1;
%   pusch.PTRS.FrequencyDensity = 2;
%   pusch.PTRS.REOffset = '01';
%   ind = nrPUSCHPTRSIndices(carrier,pusch);
%
%   Example 2:
%   % Generate PT-RS indices of a physical uplink shared channel occupying
%   % 10 MHz bandwidth for a 15 kHz SCS carrier, with transform precoding
%   % set to 1. Configure DM-RS with length set to 1, type A position set
%   % to 2, number of additional positions set to 0, and configuration type
%   % set to 1. Enable PT-RS with number of PT-RS samples set to 2, and
%   % number of PT-RS groups set to 4.
%
%   carrier = nrCarrierConfig;
%   pusch = nrPUSCHConfig('TransformPrecoding',1);
%   pusch.DMRS.DMRSLength = 1;
%   pusch.DMRS.DMRSTypeAPosition = 2;
%   pusch.DMRS.DMRSAdditionalPosition = 0;
%   pusch.DMRS.DMRSConfigurationType = 1;
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.NumPTRSSamples = 2;
%   pusch.PTRS.NumPTRSGroups = 4;
%   ind = nrPUSCHPTRSIndices(carrier,pusch);
%
%   Example 3:
%   % Generate and plot PT-RS indices of a physical uplink shared channel
%   % occupying first 5 resource blocks of a 30 kHz SCS carrier having
%   % 5 MHz transmission bandwidth. Enable intra-slot frequency hopping
%   % with starting resource block of second hop at 4. Configure DM-RS with
%   % number of additional positions set to 1. Enable PT-RS with frequency
%   % density set to 4, and time density set to 2.
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
%   pusch.EnablePTRS = 1;
%   pusch.PTRS.TimeDensity = 2;
%   pusch.PTRS.FrequencyDensity = 4;
%
%   % Get the resource element indices
%   ind = nrPUSCHPTRSIndices(carrier,pusch);
%
%   % Plot the indices on the grid
%   grid = complex(zeros([carrier.NSizeGrid*12 carrier.SymbolsPerSlot pusch.NumLayers]));
%   grid(ind) = 1;
%   imagesc(abs(grid(:,:,1)));
%   axis xy;
%   xlabel('OFDM symbols');
%   ylabel('Subcarriers');
%   title('PUSCH PT-RS resource elements in the carrier resource grid');
%
%   See also nrPUSCHPTRS, nrPUSCHDMRSIndices, nrPUSCHIndices,
%   nrPUSCHConfig, nrPUSCHDMRSConfig, nrPUSCHPTRSConfig, nrCarrierConfig,
%   nrIntraCellGuardBandsConfig.

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

    % Capture set of transmission layers required
    nLayers = double(pusch.NumLayers);
    if isempty(pusch.DMRS.DMRSPortSet)
        layers = 0:nLayers-1;
    else
        layers = double(pusch.DMRS.DMRSPortSet);
    end
    nRBSC = 12; % Number of subcarriers in a resource block

    % Get the number of PT-RS ports
    if ~pusch.TransformPrecoding
        % PT-RS port set
        if isempty(pusch.PTRS.PTRSPortSet)
            ptrsPorts = min(layers(:));
        else
            ptrsPorts = double(unique(pusch.PTRS.PTRSPortSet(:)));
        end
    else
        ptrsPorts = layers;
    end
    nPTRSPorts = numel(ptrsPorts);

    % Get the number of antenna ports
    codebookTxFlag = strcmpi(pusch.TransmissionScheme,'codebook');
    if codebookTxFlag
        nports = double(pusch.NumAntennaPorts);
    else
        nports = nLayers;
    end

    % Cache the number of columns in the output
    ncols = nPTRSPorts;
    if ~pusch.TransformPrecoding && codebookTxFlag
        ncols = nports;
    end

    % Get prbset, symbolset and dmrssymbols
    [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(carrier,pusch,nSizeBWP,ftable);

    % Check the presence of PT-RS
    if ~pusch.EnablePTRS || isempty(dmrssymbols) || isempty(prbset)
        % Return empty output, when either EnablePTRS is set to 0, prbset
        % is empty or dmrssymbols is empty
        ind = zeros(0,ncols);
    else
        % PT-RS OFDM symbol set
        ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(symbolset,dmrssymbols,double(pusch.PTRS.TimeDensity));

        if ~pusch.TransformPrecoding
            % Transform precoding disabled

            % Subcarrier locations of PT-RS for OFDM symbol of each hop
            kPTRS = double(pusch.PTRS.FrequencyDensity);
            ptrsREOffset = pusch.PTRS.REOffset;
            fhop = ~strcmpi(freqHopping,'neither');
            subInd = nr5g.internal.pxsch.ptrsSubcarrierIndicesCPOFDM(prbset,pusch.DMRS.DMRSConfigurationType,kPTRS,ptrsREOffset,ptrsPorts,pusch.RNTI,fhop,double(pusch.SecondHopStartPRB));

            % Find PT-RS linear indices relative to the carrier resource
            % grid, by marking the locations of PT-RS in the grid
            grid = zeros([nSizeGrid*nRBSC symbperslot nLayers]);
            for p = 1:nPTRSPorts
                portIndex = find(ptrsPorts(p) == layers(:));
                if ~fhop
                    % FrequencyHopping set to 'neither'
                    grid(subInd{p}(:,1)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(:)+1,portIndex(1)) = 1;
                elseif strcmpi(freqHopping,'interSlot')
                    % FrequencyHopping set to 'interSlot'
                    if mod(nslot,2) == 1
                        colIndex = 2;
                    else
                        colIndex = 1;
                    end
                    grid(subInd{p}(:,colIndex)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(:)+1,portIndex(1)) = 1;
                else
                    % FrequencyHopping set to 'intraSlot'
                    dmrsIndex = dmrssymbols >= (floor(pusch.SymbolAllocation(end)/2)+pusch.SymbolAllocation(1));
                    index = ptrssymbols >= (floor(pusch.SymbolAllocation(end)/2)+pusch.SymbolAllocation(1));
                    if ~isempty(dmrssymbols(~dmrsIndex))
                        grid(subInd{p}(:,1)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(~index)+1,portIndex(1)) = 1;
                    end
                    if ~isempty(dmrssymbols(dmrsIndex))
                        grid(subInd{p}(:,2)+nRBSC*(nStartBWP-nStartGrid)+1,ptrssymbols(index)+1,portIndex(1)) = 1;
                    end
                end
            end
            if codebookTxFlag
                % Codebook based transmission scheme
                sgrid = sum(grid,3);
                ptrsIndNorm = find(sgrid)-1;
                ind = repmat(ptrsIndNorm(:),1,double(pusch.NumAntennaPorts)) + repmat((nRBSC*symbperslot*nSizeGrid)*(0:ncols-1),numel(ptrsIndNorm),1);
            else
                % Non-codebook based transmission scheme
                ind = reshape(find(grid)-1,[],ncols);
            end
        else
            % Transform precoding enabled
            symInd = nr5g.internal.pusch.ptrsSymIndicesDFTsOFDM(symbolset,dmrssymbols,ptrssymbols);
            scInd = nr5g.internal.pusch.ptrsSCIndicesDFTsOFDM(double(pusch.PTRS.NumPTRSSamples),double(pusch.PTRS.NumPTRSGroups),numel(prbset)*12);
            puschAlloc = zeros([numel(prbset)*12 numel(symbolset)-numel(dmrssymbols) nLayers]);
            puschAlloc(scInd+1,symInd+1,:) = 1;
            ind = reshape(find(puschAlloc)-1,[],ncols);
        end
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPUSCHPTRSIndices';
        opts = nr5g.internal.parseOptions(fcnName,...
            {'IndexStyle','IndexBase','IndexOrientation'},varargin{:});

        % Apply PV pairs
        if ~pusch.TransformPrecoding
            ind = nr5g.internal.applyIndicesOptions(...
                [nSizeGrid*nRBSC symbperslot nports],opts,ind(:));
            ind = nr5g.internal.applyIndexOrientation(...
                [nStartGrid nSizeGrid], [nStartBWP nSizeBWP],symbperslot,opts,ind);
        else
            ind = nr5g.internal.applyIndicesOptions(...
                [numel(prbset)*12 numel(symbolset)-numel(dmrssymbols) nLayers],opts,ind(:));
        end
        if strcmpi(opts.IndexStyle,'index')
            % Reshape the output for linear indexing
            if isempty(ind)
                % This statement and code is needed for code generation
                % as reshape cannot handle resizing empty inputs
                ind = repmat(uint32(0),0,ncols);
            else
                ind = reshape(ind,[],ncols);
            end
        end
    else
        % 1-based
        ind = uint32(ind + 1);
    end

end